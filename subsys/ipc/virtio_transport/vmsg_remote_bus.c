/*
 * Copyright (c) 2024 STMicroelectronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/ipc/virtio_transport.h>
#include <openamp/open_amp.h>

#define DT_DRV_COMPAT	zephyr_ipc_openamp_virtio_msg

#define NUM_INSTANCES	DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

#define WQ_STACK_SIZE	1024

#define	PRIO_COOP	BIT(0)
#define	PRIO_PREEMPT	BIT(1)

#if defined(CONFIG_THREAD_MAX_NAME_LEN)
#define THREAD_MAX_NAME_LEN CONFIG_THREAD_MAX_NAME_LEN
#else
#define THREAD_MAX_NAME_LEN 1
#endif

#define LOG_MODULE_NAME virtio_msg_bus
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_VIRTIO_MSG_SERVICE_LOG_LEVEL);

K_THREAD_STACK_ARRAY_DEFINE(mbox_stack, NUM_INSTANCES, WQ_STACK_SIZE);

#define HOST IS_ENABLED(CONFIG_VIRTIO_MSG_SERVICE_MODE_HOST)
#if HOST
#error virtio msg Host mode not yet supported !!!
#endif

#define	VMSG_STATE_OPENED	BIT(0)
#define	VMSG_STATE_CONNECTED	BIT(1)

/**
 * Structure identifying a bus device to be probed
 */
struct vmsgbus_child_dev {
	const struct device *dev;
	struct virtio_msg_device vmdev;
};

/** List of virtio devices attached to a bus */
struct vmsgbus_child_dev_list {
	/** Identifiers for children of the node */
	const struct vmsgbus_child_dev *children;
	/** Number of children of the node */
	unsigned int num_children;
};

struct virtio_msg_bus_config_t {
	uint32_t bus_base;
	int bus_size;
	unsigned int role;
	uintptr_t shm_addr;
	size_t shm_size;
	unsigned int wq_prio_type;
	unsigned int wq_prio;
	unsigned int id;
};

#define DEV_CFG(dev)	\
	((const struct virtio_msg_bus_config_t *)(dev)->config)

struct vmsgbus_data_t {
	const struct virtio_msg_bus_config_t  *conf;
	struct virtio_msg_bus_device	vmsg_bus;
	struct virtio_msg_bus_cfg	vmsg_bus_cfg;
	const struct vmsgbus_child_dev_list *dev_list;
	int				state;

	/* MBOX WQ */
	struct k_work mbox_work;
	struct k_work_q mbox_wq;

	struct k_sem			sem;
	struct k_thread			thread;
	k_thread_stack_t		*thread_stack;
};

#define DEV_DATA(dev)	\
	((struct vmsgbus_data_t *)(dev)->data)

static const struct device *const ipm_handle =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static void mbox_callback_process(struct k_work *item)
{
	struct vmsgbus_data_t *data;

	data = CONTAINER_OF(item, struct vmsgbus_data_t, mbox_work);

	LOG_DBG("message received dev ID 0x%08X", data->conf->id);

	virtio_msg_bus_receive(&data->vmsg_bus);
}

static void ipm_callback(const struct device *ipmdev, void *user_data,
			 uint32_t id, volatile void *msg_data)
{
	struct vmsgbus_data_t *data = user_data;

	k_work_submit_to_queue(&data->mbox_wq, &data->mbox_work);
}

static int mbox_init(const struct device *dev)
{
	const struct virtio_msg_bus_config_t *conf = DEV_CFG(dev);
	struct vmsgbus_data_t *data = DEV_DATA(dev);
	struct k_work_queue_config wq_cfg = {.name = dev->name};
	int prio, err;

	if (!ipm_handle) {
		LOG_ERR("Only IPM supported yet!");
	}

	prio = (conf->wq_prio_type == PRIO_COOP) ? K_PRIO_COOP(conf->wq_prio) :
						   K_PRIO_PREEMPT(conf->wq_prio);

	k_work_queue_init(&data->mbox_wq);
	k_work_queue_start(&data->mbox_wq, mbox_stack[conf->id], WQ_STACK_SIZE, prio, &wq_cfg);

	if (IS_ENABLED(CONFIG_THREAD_NAME)) {
		char name[THREAD_MAX_NAME_LEN];

		snprintk(name, sizeof(name), "mbox_wq #%d", conf->id);
		k_thread_name_set(&data->mbox_wq.thread, name);
	}

	k_work_init(&data->mbox_work, mbox_callback_process);

	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready");
		return -ENODEV;
	}

	ipm_register_callback(ipm_handle, ipm_callback, data);

	err = ipm_set_enabled(ipm_handle, 1);
	if (err != 0) {
		LOG_ERR("Could not enable IPM interrupts and callbacks");
		return err;
	}

	return err;
}

/**
 * Open the backend dev callback.
 */
static struct virtio_device *vmsg_bus_register_dev(const struct device *bus,
						   const struct device *dev,
						   const struct virtio_device_id *dev_id,
						   unsigned int role,  void **token)
{
	struct vmsgbus_data_t *data = DEV_DATA(bus);
	const struct vmsgbus_child_dev_list *list = data->dev_list;
	struct virtio_msg_device *vmdev = NULL;
	unsigned int i;
	int err;

	for (i = 0; i < list->num_children; i++) {
		if (list->children[i].dev == dev) {
			vmdev = (struct virtio_msg_device *)&list->children[i].vmdev;
			vmdev->bus_id = i;
			break;
		}
		i++;
	}

	if (!vmdev)
		return NULL;

	LOG_DBG("register virtio dev 0x%08X on virtio bus v 0x%08X",
		(uint32_t)dev, (uint32_t)bus);

	err = virtio_msg_bus_register_vdev(&data->vmsg_bus, vmdev, dev_id, role);
	if (err)
		return  NULL;

	*token = vmdev;

	return &vmdev->vdev;
}

/**
 * Open the backend dev callback.
 */
static int vmsg_bus_open(const struct device *dev)
{
	const struct virtio_msg_bus_config_t *conf = DEV_CFG(dev);
	struct vmsgbus_data_t *data = DEV_DATA(dev);
	int err;

	LOG_DBG("Open dev 0x%08X", (uint32_t)dev);

	if (conf->role == VIRTIO_DEV_DRIVER) {
		LOG_ERR("VIRTIO DRIVER ROLE not yet supported");
		return -1;
	}

	err = mbox_init(dev);
	if (err != 0)
		return err;

	data->state = VMSG_STATE_OPENED;

	/* Inform the remote side that we are ready to communicatye */
	err = virtio_msg_bus_connect(&data->vmsg_bus, 100);
	if (!err)
		data->state |= VMSG_STATE_CONNECTED;

	return 0;
}

static const struct ipc_virtio_transport vmsg_bus_ops = {
	.open_instance = vmsg_bus_open,
	.register_virtio = vmsg_bus_register_dev,
	.close_instance = NULL, /* not implemented */
	.send = NULL,
};

static int vmsg_bus_notify(struct virtio_msg_bus_device *devbus)
{
	/* TODO:
	 * should rely on mailbox instead of IPM for multi dev support (channel ID fixed to 4)
	 */
	return ipm_send(ipm_handle, 0, 4, NULL, 0);
}

static int vmsg_bus_init(const struct device *dev)
{
	const struct virtio_msg_bus_config_t *conf = DEV_CFG(dev);
	struct vmsgbus_data_t *data = DEV_DATA(dev);
	int32_t err;

	LOG_DBG("virtio msg service initialization start");

	data->vmsg_bus_cfg.msg_paddr = conf->bus_base;
	data->vmsg_bus_cfg.msg_vaddr = conf->bus_base;
	data->vmsg_bus_cfg.msg_size = conf->shm_size;
	data->vmsg_bus_cfg.shm_paddr = conf->shm_addr;
	data->vmsg_bus_cfg.shm_vaddr = conf->shm_addr;
	data->vmsg_bus_cfg.shm_size = conf->shm_size;
	data->vmsg_bus_cfg.notify = vmsg_bus_notify;
	data->vmsg_bus_cfg.role = VIRTIO_DEV_DEVICE;
	data->conf = conf;
	data->state  = 0;

	err = virtio_msg_bus_init(&data->vmsg_bus, &data->vmsg_bus_cfg, &data);
	if (err) {
		LOG_ERR("virtio msg init failed with error %d", err);
		return err;
	}

	return 0;
}

#if defined(CONFIG_ARCH_POSIX)
#define VMSGBUS_PRE(i) extern char IPC##i##_shm_buffer[];
#define VMSGBUS_SHM_ADDR(i) (const uintptr_t)IPC##i##_shm_buffer
#else
#define VMSGBUS_PRE(i)
#define VMSGBUS_SHM_ADDR(i) DT_REG_ADDR(DT_INST_PHANDLE(i, memory_region))
#endif /* defined(CONFIG_ARCH_POSIX) */

#define DECLARE_VIRTIO_DEVICE(node_id)							\
	{										\
		.dev = DEVICE_DT_GET(node_id),						\
	},

#define DEFINE_VMSGBUS_DEVICE(i)							\
	VMSGBUS_PRE(i)									\
	struct vmsgbus_child_dev vmsgbus_##i##_dev_list[] = {				\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(i,					\
					     DECLARE_VIRTIO_DEVICE)			\
	};										\
	static const struct vmsgbus_child_dev_list vmsgbus_child_dev_list_##i = {	\
		.children = vmsgbus_##i##_dev_list,					\
		.num_children = ARRAY_SIZE(vmsgbus_##i##_dev_list),			\
	};										\
	static struct virtio_msg_bus_config_t vmsgbus_config_##i = {			\
		.bus_base = DT_INST_REG_ADDR(i),					\
		.bus_size = DT_INST_REG_SIZE(i),					\
		.role = DT_ENUM_IDX_OR(DT_DRV_INST(i), role, VIRTIO_DEV_DEVICE),	\
		.shm_size = DT_REG_SIZE(DT_INST_PHANDLE(i, memory_region)),		\
		.shm_addr = VMSGBUS_SHM_ADDR(i),					\
		.wq_prio = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, zephyr_priority),	\
			   (DT_INST_PROP_BY_IDX(i, zephyr_priority, 0)),		\
			   (0)),							\
		.wq_prio_type = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, zephyr_priority),	\
			   (DT_INST_PROP_BY_IDX(i, zephyr_priority, 1)),		\
			   (PRIO_PREEMPT)),						\
		.id = i,								\
	};										\
											\
	static struct vmsgbus_data_t vmsgbus_data_##i = {				\
		.dev_list = &vmsgbus_child_dev_list_##i,				\
	};										\
											\
	DEVICE_DT_INST_DEFINE(i,							\
			 &vmsg_bus_init,						\
			 NULL,								\
			 &vmsgbus_data_##i,						\
			 &vmsgbus_config_##i,						\
			 POST_KERNEL,							\
			 CONFIG_VIRTIO_MSG_SERVICE_INIT_PRIORITY,			\
			 &vmsg_bus_ops);
DT_INST_FOREACH_STATUS_OKAY(DEFINE_VMSGBUS_DEVICE)
