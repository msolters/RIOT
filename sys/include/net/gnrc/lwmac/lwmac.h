/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lwmac Simplest possible MAC layer
 * @ingroup     net
 * @brief       Lightweight MAC protocol that allows for duty cycling to save
 *              energy.
 * @{
 *
 * @file
 * @brief       Interface definition for the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_LWMAC_H
#define GNRC_LWMAC_H

#include "kernel.h"
#include "net/gnrc/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set the default message queue size for LWMAC layer
 */
#ifndef LWMAC_IPC_MSG_QUEUE_SIZE
#define LWMAC_IPC_MSG_QUEUE_SIZE        (8U)
#endif

/**
 * @brief   Count of parallel timeouts. Shouldn't needed to be changed.
 */
#ifndef LWMAC_TIMEOUT_COUNT
#define LWMAC_TIMEOUT_COUNT             (3U)
#endif

/**
 * @brief   Count of nodes in one-hop distance whose wakeup phase is tracked
 */
#ifndef LWMAC_NEIGHBOUR_COUNT
#define LWMAC_NEIGHBOUR_COUNT           (8U)
#endif

/**
 * @brief   Set the default queue size for packets coming from higher layers
 */
#ifndef LWMAC_TX_QUEUE_SIZE
#define LWMAC_TX_QUEUE_SIZE             (8U)
#endif

/**
 * @brief   Set the default queue size for incoming packets
 */
#ifndef LWMAC_RX_QUEUE_SIZE
#define LWMAC_RX_QUEUE_SIZE             (8U)
#endif

#ifndef LWMAC_WAKEUP_INTERVAL_MS
#define LWMAC_WAKEUP_INTERVAL_MS        (100U)
#endif

#ifndef LWMAC_TIME_BETWEEN_WR_US
#define LWMAC_TIME_BETWEEN_WR_US        (7000U)
#endif

#ifndef LWMAC_WAKEUP_DURATION_MS
#define LWMAC_WAKEUP_DURATION_MS        (LWMAC_TIME_BETWEEN_WR_US / 1000 * 2)
#endif

/* Start sending earlier then known phase. Therefore advance to beginning edge
 * of destinations wakeup phase over time.
 * Note: * RTT tick is ~30us
 *       * there is a certain overhead until WR will be sent
 */
#ifndef LWMAC_WR_BEFORE_PHASE_US
#define LWMAC_WR_BEFORE_PHASE_US        (500U)
#endif

/* WR preparation overhead before it can be sent (higher with debugging output) */
#ifndef LWMAC_WR_PREPARATION_US
#define LWMAC_WR_PREPARATION_US         (7000U + LWMAC_WR_BEFORE_PHASE_US)
#endif

/* How long to wait after a WA for data to come in. It's enough to catch the
 * beginning of the packet if the transceiver supports RX_STARTED event (this
 * can be important for big packets). */
#ifndef LWMAC_DATA_DELAY_US
#define LWMAC_DATA_DELAY_US             (5000U)
#endif

/* Max link layer address length in bytes */
#ifndef LWMAC_MAX_L2_ADDR_LEN
#define LWMAC_MAX_L2_ADDR_LEN           (2U)
#endif


/**
 * @brief   Initialize an instance of the LWMAC layer
 *
 * The initialization starts a new thread that connects to the given netdev
 * device and starts a link layer event loop.
 *
 * @param[in] stack         stack for the control thread
 * @param[in] stacksize     size of *stack*
 * @param[in] priority      priority for the thread housing the LWMAC instance
 * @param[in] name          name of the thread housing the LWMAC instance
 * @param[in] dev           netdev device, needs to be already initialized
 *
 * @return                  PID of LWMAC thread on success
 * @return                  -EINVAL if creation of thread fails
 * @return                  -ENODEV if *dev* is invalid
 */
kernel_pid_t gnrc_lwmac_init(char *stack, int stacksize, char priority,
                           const char *name, gnrc_netdev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_H */
/** @} */
