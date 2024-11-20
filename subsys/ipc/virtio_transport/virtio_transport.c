/*
 * Copyright (c) 2024, STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ipc/virtio_transport.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

LOG_MODULE_REGISTER(ipc_virtio_transport, CONFIG_IPC_SERVICE_LOG_LEVEL);

struct virtio_device *ipc_virtio_transport_register_device(const struct device *bus,
							   const struct device *dev,
							   const struct virtio_device_id *dev_id,
							   unsigned int role, void **token)
{
	const struct  ipc_virtio_transport *backend;

	if (!bus | !dev) {
		LOG_ERR("Invalid bus");
		return NULL;
	}

	backend = (const struct  ipc_virtio_transport *)bus->api;

	if (!backend) {
		LOG_ERR("Invalid backend configuration");
		return NULL;
	}

	if (!backend->register_virtio) {
		/* maybe not needed on backend */
		return NULL;
	}

	return backend->register_virtio(bus, dev, dev_id, role, token);
}

int  ipc_virtio_transport_open_instance(const struct device *instance)
{
	const struct  ipc_virtio_transport *backend;

	if (!instance) {
		LOG_ERR("Invalid instance");
		return -EINVAL;
	}

	backend = (const struct  ipc_virtio_transport *)instance->api;

	if (!backend) {
		LOG_ERR("Invalid backend configuration");
		return -EIO;
	}

	if (!backend->open_instance) {
		/* maybe not needed on backend */
		return 0;
	}

	return backend->open_instance(instance);
}

int  ipc_virtio_transport_close_instance(const struct device *instance)
{
	const struct  ipc_virtio_transport *backend;

	if (!instance) {
		LOG_ERR("Invalid instance");
		return -EINVAL;
	}

	backend = (const struct  ipc_virtio_transport *)instance->api;

	if (!backend) {
		LOG_ERR("Invalid backend configuration");
		return -EIO;
	}

	if (!backend->close_instance) {
		/* maybe not needed on backend */
		return 0;
	}

	return backend->close_instance(instance);
}

