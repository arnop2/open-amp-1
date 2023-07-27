/*
 * Copyright (c) 2024, STMICROLECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_virtio_i2c_dev

#include <zephyr/ipc/virtio_transport.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <openamp/open_amp.h>

struct i2c_virtio_config_t {
	const struct device	*dev;
	const struct device	*parent;
	unsigned int		bus_id;
	unsigned int		type;
};

#define DEV_CFG(dev)	\
	((struct i2c_virtio_config_t *)(dev)->config)

struct i2c_virtio_data_t {
	struct virtio_i2c_dev		i2c_dev;
	struct i2c_target_config	*broadcast;

	sys_slist_t			targets;

	struct k_sem			sem;
	struct k_thread			thread;
	k_thread_stack_t		*thread_stack;
	unsigned int			state;

	/** Backend-specific token used to identify an endpoint in an instance. */
	void *token;

};

#define DEV_DATA(dev)	\
	((struct i2c_virtio_data_t * const)(dev)->data)

#define I2C_VIRTIO_STACK_SIZE	512

static struct i2c_target_config *find_address(struct i2c_virtio_data_t *data,
					      uint16_t address)
{
	struct i2c_target_config *cfg = NULL;
	sys_snode_t *node;

	SYS_SLIST_FOR_EACH_NODE(&data->targets, node) {
		cfg = CONTAINER_OF(node, struct i2c_target_config, node);

		if (cfg->address == address)
			return cfg;
	}

	return NULL;
}

static int i2c_virtio_bus_callback(void *callback_data,
				   struct virtio_i2c_msg *msg)
{
	struct i2c_virtio_data_t *data = callback_data;
	struct i2c_target_config *cfg;
	uint8_t *ptr;
	uint32_t len;
	int ret = EINVAL;;

	if (data->broadcast) {
		data->broadcast->address = msg->addr;
		cfg = data->broadcast;
	} else
		cfg = find_address(data, msg->addr);

	if (!cfg)
		return -EINVAL;

	if (msg->flags == VIRTIO_I2C_MSG_READ) {
		if (cfg->callbacks->buf_read_requested)
			ret = cfg->callbacks->buf_read_requested(cfg, &ptr, &len);
		if (ret || len < msg->len)
			return ret;
		memcpy(msg->buf, ptr, msg->len);
	} else {
		if (cfg->callbacks->buf_write_received)
			cfg->callbacks->buf_write_received(cfg, msg->buf, msg->len);
	}

	return 0;
}

static int i2c_virtio_runtime_configure(const struct device *dev,
					uint32_t config)
{
	return 0;
}

static int i2c_virtio_target_register(const struct device *dev,
				      struct i2c_target_config *config)
{
	struct i2c_virtio_data_t *data = DEV_DATA(dev);
	static volatile int to = 1;
	
	while(!to);

	if (!config)
		return -EINVAL;

	/* if address is 0xff then it means that any address is allowed */
	if (config->address == 0xff) {
		if (data->broadcast)
			return -EINVAL;

		data->broadcast = config;
		return 0;
	}

	if (config->address == 0xff)
		data->broadcast = config;

	/* Check the address is unique */
	if (find_address(data, config->address))
		return -EINVAL;

	sys_slist_append(&data->targets, &config->node);

	return 0;
}

static int i2c_virtio_target_unregister(const struct device *dev,
					struct i2c_target_config *config)
{
	struct i2c_virtio_data_t *data = DEV_DATA(dev);

	if (!config)
		return -EINVAL;

	if (config == data->broadcast) {
		if (!data->broadcast)
			return -EINVAL;

		data->broadcast->address = 0xff;

		data->broadcast = NULL;
		return 0;
	}

	if (!sys_slist_find_and_remove(&data->targets, &config->node))
		return -EINVAL;

	return 0;
}

static void i2c_virtio_it_callback(struct virtqueue *vq)
{
	struct virtio_i2c_dev *i2c_dev = vq->priv;
	struct i2c_virtio_data_t *data = CONTAINER_OF(i2c_dev, struct i2c_virtio_data_t, i2c_dev);

	/* notify the thread that a buffer is available */
	k_sem_give(&data->sem);
}


static void i2c_virtio_task(void *u1,
			    void *u2,
			    void *u3)
{
	struct i2c_virtio_data_t *data = u1;

rst:
 	/** wait for the virtio device to be fully initialized */
	while (!virtio_i2c_ready(&data->i2c_dev))
		k_msleep(10);

	/** The thread will keep waiting for available buffers
	 * We need this thread to get out of the interruption context
	 */
	while (virtio_i2c_ready(&data->i2c_dev)) {

		/** when the semaphore is given we have a buffer available */
		k_sem_take(&data->sem, K_FOREVER);

		virtio_i2c_handle_avail(&data->i2c_dev);
	}

	goto rst;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_virtio_runtime_configure,
	.target_register = i2c_virtio_target_register,
	.target_unregister = i2c_virtio_target_unregister,
};

static const struct virtio_device_id  i2c_virtio_dev_id = {
	. device = VIRTIO_ID_I2C_ADAPTER,
};

static int i2c_virtio_dev_init(const struct device *dev)
{
	struct i2c_virtio_config_t *conf = DEV_CFG(dev);
	struct i2c_virtio_data_t *data = DEV_DATA(dev);
	struct virtio_i2c_config vi2c_conf;
	static struct virtio_device *vdev;
	int ret;

	if (!device_is_ready(conf->parent)) {
		return -ENODEV;
	}

	/** list used for the I2C target api */
	sys_slist_init(&data->targets);

	/** once everything is ready we start the thread */
	k_sem_init(&data->sem, 0, 1);

	/* register the virtio device on the virtio transport */
	vdev = ipc_virtio_transport_register_device(conf->parent, conf->dev, &i2c_virtio_dev_id,
						   VIRTIO_DEV_DEVICE, &data->token);
	if (!vdev)
		return -ENODEV;

	data->i2c_dev.vdev = vdev;

	/** Configure the i2c device */
	vi2c_conf.cb = i2c_virtio_bus_callback;
	vi2c_conf.cb_data = data;

	vi2c_conf.bus_id = conf->bus_id;

	ret = virtio_i2c_configure(&data->i2c_dev, i2c_virtio_it_callback, &vi2c_conf);
	if (ret)
		return ret;

	k_thread_create(&data->thread,
			data->thread_stack,
			I2C_VIRTIO_STACK_SIZE,
			i2c_virtio_task,
			data,
			conf,
			NULL,
			K_PRIO_COOP(7),
			0,
			K_NO_WAIT);

	return 0;
}

#define I2C_DEVICE_INIT_VIRTIO(n)						\
	K_THREAD_STACK_DEFINE(i2c_virtio_stack_##id, I2C_VIRTIO_STACK_SIZE);	\
										\
	static struct i2c_virtio_config_t i2c_virtio_config_##n = {		\
		.dev = DEVICE_DT_INST_GET(n),					\
		.parent = DEVICE_DT_GET(DT_PARENT(DT_INST(n, DT_DRV_COMPAT))),	\
		.bus_id = DT_INST_REG_ADDR(n),					\
		.type = DT_PROP(DT_DRV_INST(0), device_id),			\
	};									\
	static struct i2c_virtio_data_t i2c_virtio_data_##n = {			\
		.thread_stack = i2c_virtio_stack_##id,				\
	};									\
										\
	I2C_DEVICE_DT_INST_DEFINE(n,						\
			i2c_virtio_dev_init, NULL,				\
			&i2c_virtio_data_##n,					\
			&i2c_virtio_config_##n,					\
			POST_KERNEL,						\
			50,							\
			&api_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_DEVICE_INIT_VIRTIO)
