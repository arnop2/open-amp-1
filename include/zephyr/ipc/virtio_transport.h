/*
 * Copyright (c) 2024 STMicroelectronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_IPC_VIRTIO_TRANSPORT_H_
#define ZEPHYR_INCLUDE_IPC_VIRTIO_TRANSPORT_H_

#include <zephyr/kernel.h>
#include <stdio.h>

#include <openamp/open_amp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Virtio transport
 * @defgroup ipc_virtio_transport IPC virtio transport
 * @ingroup ipc
 * @{
 */

/** @brief IPC virtio transport configuration structure.
 *
 *  This structure is used for configuration during registration.
 */
struct ipc_virtio_transport {
	/** @brief Pointer to the function that will be used to open an instance
	 *
	 *  @param[in] instance Instance pointer.
	 *
	 *  @retval -EALREADY when the instance is already opened.
	 *
	 *  @retval 0 on success
	 *  @retval other errno codes depending on the implementation of the
	 *	    backend.
	 */
	int (*open_instance)(const struct device *instance);

	/** @brief Pointer to the function that will be used to close an instance
	 *
	 *  @param[in] instance Instance pointer.
	 *
	 *  @retval -EALREADY when the instance is not already inited.
	 *
	 *  @retval 0 on success
	 *  @retval other errno codes depending on the implementation of the
	 *	    backend.
	 */
	int (*close_instance)(const struct device *instance);

	/** @brief Pointer to the function that will be used to send data to the endpoint.
	 *
	 *  @param[in] instance Instance pointer.
	 *  @param[in] token Backend-specific token.
	 *  @param[in] data Pointer to the buffer to send.
	 *  @param[in] len Number of bytes to send.
	 *
	 *  @retval -EINVAL when instance is invalid.
	 *  @retval -ENOENT when the endpoint is not registered with the instance.
	 *  @retval -EBADMSG when the message is invalid.
	 *  @retval -EBUSY when the instance is busy or not ready.
	 *  @retval -ENOMEM when no memory / buffers are available.
	 *
	 *  @retval bytes number of bytes sent.
	 *  @retval other errno codes depending on the implementation of the
	 *	    backend.
	 */
	int (*send)(const struct device *instance, void *token,
		    const void *data, size_t len);

	/** @brief Pointer to the function that will be used to register endpoints.
	 *
	 *  @param[in] instance Instance to register the endpoint onto.
	 *  @param[out] token Backend-specific token.
	 *  @param[in] cfg Endpoint configuration.
	 *
	 *  @retval -EINVAL when the endpoint configuration or instance is invalid.
	 *  @retval -EBUSY when the instance is busy or not ready.
	 *
	 *  @retval 0 on success
	 *  @retval other errno codes depending on the implementation of the
	 *	    backend.
	 */
	struct virtio_device *(*register_virtio)(const struct device *instance,
						 const struct device *dev,
						 const struct virtio_device_id *dev_id,
						 unsigned int role, void **token);

	/** @brief Pointer to the function that will be used to deregister endpoints
	 *
	 *  @param[in] instance Instance from which to deregister the endpoint.
	 *  @param[in] token Backend-specific token.
	 *
	 *  @retval -EINVAL when the endpoint configuration or instance is invalid.
	 *  @retval -ENOENT when the endpoint is not registered with the instance.
	 *  @retval -EBUSY when the instance is busy or not ready.
	 *
	 *  @retval 0 on success
	 *  @retval other errno codes depending on the implementation of the
	 *      backend.
	 */
	int (*deregister_virtio)(const struct device *instance, void *token);
};

struct virtio_device *ipc_virtio_transport_register_device(const struct device *bus,
							   const struct device *dev,
							   const struct virtio_device_id *dev_id,
							   unsigned int role, void **token);

/** @brief Open an instance
 *
 *  Function to be used to open an instance before being able to register a new
 *  endpoint on it.
 *
 *  @param[in] instance Instance to open.
 *
 *  @retval -EINVAL when instance configuration is invalid.
 *  @retval -EIO when no backend is registered.
 *  @retval -EALREADY when the instance is already opened (or being opened).
 *
 *  @retval 0 on success or when not implemented on the backend (not needed).
 *  @retval other errno codes depending on the implementation of the backend.
 */
int  ipc_virtio_transport_open_instance(const struct device *instance);

/** @brief Close an instance
 *
 *  Function to be used to close an instance. All bounded endpoints must be
 *  deregistered using  ipc_virtio_transport_deregister_endpoint before this
 *  is called.
 *
 *  @param[in] instance Instance to close.
 *
 *  @retval -EINVAL when instance configuration is invalid.
 *  @retval -EIO when no backend is registered.
 *  @retval -EALREADY when the instance is not already opened.
 *  @retval -EBUSY when an endpoint exists that hasn't been
 *           deregistered
 *
 *  @retval 0 on success or when not implemented on the backend (not needed).
 *  @retval other errno codes depending on the implementation of the backend.
 */
int  ipc_virtio_transport_close_instance(const struct device *instance);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_IPC_VIRTIO_TRANSPORT_H_ */
