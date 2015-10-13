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
 * @brief       Internal types of LWMAC
 * @{
 *
 * @file
 * @brief       Internal types used by the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_LWMAC_TYPES_H_
#define GNRC_LWMAC_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <kernel.h>
#include <vtimer.h>
#include <net/gnrc.h>
#include <net/gnrc/lwmac/lwmac.h>
#include <net/gnrc/lwmac/hdr.h>
#include <net/gnrc/lwmac/packet_queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/

#define LWMAC_EVENT_RTT_TYPE            (0x4300)
#define LWMAC_EVENT_RTT_START           (0x4301)
#define LWMAC_EVENT_RTT_STOP            (0x4302)
#define LWMAC_EVENT_RTT_PAUSE           (0x4303)
#define LWMAC_EVENT_RTT_RESUME          (0x4304)
#define LWMAC_EVENT_RTT_WAKEUP_PENDING  (0x4305)
#define LWMAC_EVENT_RTT_SLEEP_PENDING   (0x4306)
#define LWMAC_EVENT_TIMEOUT_TYPE        (0x4400)

/******************************************************************************/

typedef enum {
    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,
    LISTENING,
    RECEIVING,      /* RX is handled in own state machine */
    TRANSMITTING,   /* TX is handled in own state machine */
    SLEEPING,
    STATE_COUNT
} lwmac_state_t;

/******************************************************************************/

typedef enum {
    TX_STATE_STOPPED = 0,
    TX_STATE_INIT,          /**< Initiate transmission */
    TX_STATE_SEND_WR,       /**< Send a wakeup request */
    TX_STATE_WAIT_WR_SENT,  /**< Wait until WR sent to set timeout */
    TX_STATE_WAIT_FOR_WA,   /**< Wait for dest node's wakeup ackknowledge */
    TX_STATE_SEND_DATA,     /**< Send the actual payload data */
    TX_STATE_WAIT_FEEDBACK, /**< Wait if packet was ACKed */
    TX_STATE_SUCCESSFUL,    /**< Transmission has finished successfully */
    TX_STATE_FAILED         /**< Payload data couldn't be delivered to dest */
} lwmac_tx_state_t;
#define LWMAC_TX_STATE_INIT TX_STATE_STOPPED

/******************************************************************************/

typedef enum {
    RX_STATE_STOPPED = 0,
    RX_STATE_INIT,          /**< Initiate reception */
    RX_STATE_WAIT_FOR_WR,   /**< Wait for a wakeup request */
    RX_STATE_SEND_WA,       /**< Send wakeup ackknowledge to requesting node */
    RX_STATE_WAIT_WA_SENT,  /**< Wait until WA sent to set timeout */
    RX_STATE_WAIT_FOR_DATA, /**< Wait for actual payload data */
    RX_STATE_SUCCESSFUL,    /**< Recption has finished successfully */
    RX_STATE_FAILED         /**< Reception over, but nothing received */
} lwmac_rx_state_t;
#define LWMAC_RX_STATE_INIT RX_STATE_STOPPED

/******************************************************************************/

typedef enum {
    TIMEOUT_DISABLED = 0,
    TIMEOUT_WR,
    TIMEOUT_NO_RESPONSE,
    TIMEOUT_WA,
    TIMEOUT_DATA,
    TIMEOUT_WAIT_FOR_DEST_WAKEUP,
} lwmac_timeout_type_t;

extern char* lwmac_timeout_names[];

/******************************************************************************/

typedef struct {
    /* Timer used for timeouts */
    vtimer_t timer;
    /* When to expire */
    timex_t interval;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    /* Lastest timeout that occurred and hasn't yet been acknowledged */
    lwmac_timeout_type_t type;
} lwmac_timeout_t;

/******************************************************************************/

typedef enum {
    TX_FEEDBACK_UNDEF = -1,
    TX_FEEDBACK_SUCCESS,
    TX_FEEDBACK_NOACK,
    TX_FEEDBACK_BUSY
} lwmac_tx_feedback_t;
#define LWMAC_TX_FEEDBACK_INIT TX_FEEDBACK_UNDEF

/******************************************************************************/

typedef struct {
    /* Internal state of reception state machine */
    lwmac_rx_state_t state;
    packet_queue_t queue;
    packet_queue_node_t _queue_nodes[LWMAC_RX_QUEUE_SIZE];
    l2_addr_t l2_addr;
    gnrc_pktsnip_t* dispatch_buffer[LWMAC_DISPATCH_BUFFER_SIZE];
} lwmac_rx_t;

#define LWMAC_RX_INIT { \
/* rx::state */             LWMAC_RX_STATE_INIT, \
/* rx::queue */             {}, \
/* rx::_queue_nodes */      {}, \
/* rx::l2_addr */           LWMAC_L2_ADDR_INIT, \
/* rx::dispatch_buffer */   {}, \
}

/******************************************************************************/

typedef struct {
    /* Address of neighbour node */
    l2_addr_t l2_addr;
    /* TX queue for this particular node */
    packet_queue_t queue;
    /* Phase relative to lwmac::last_wakeup */
    uint32_t phase;
} lwmac_tx_neighbour_t;
#define LWMAX_NEIGHBOUR_INIT        { LWMAC_L2_ADDR_INIT, {}, 0 }

#define LWMAC_PHASE_UNINITIALIZED   (0)
#define LWMAC_PHASE_MAX             (-1)

/******************************************************************************/

typedef struct {
    /* Internal state of transmission state machine */
    lwmac_tx_state_t state;
    /* TX queues for neighbouring nodes. First queue is broadcast (+1) */
    lwmac_tx_neighbour_t neighbours[LWMAC_NEIGHBOUR_COUNT + 1];
    /* Shared buffer for TX queue nodes */
    packet_queue_node_t _queue_nodes[LWMAC_TX_QUEUE_SIZE];
    /* Count how many WRs were sent until WA received */
    uint32_t wr_sent;
    /* Packet that is currently scheduled to be sent */
    gnrc_pktsnip_t* packet;
    /* Queue of destination node to which the current packet will be sent */
    lwmac_tx_neighbour_t* current_neighbour;
    uint32_t timestamp;
} lwmac_tx_t;

#define LWMAC_TX_INIT { \
/* tx::state */             LWMAC_TX_STATE_INIT, \
/* tx::neighbours */        { LWMAX_NEIGHBOUR_INIT }, \
/* tx::_queue_nodes */      {}, \
/* tx::wr_sent */           0, \
/* tx::packet */            NULL, \
/* tx::current_neighbour */ NULL, \
/* tx::timestamp */         0 \
}

/******************************************************************************/

typedef struct {
    /* PID of lwMAC thread */
    kernel_pid_t pid;
    /* NETDEV device used by lwMAC */
    gnrc_netdev_t* netdev;
    /* Internal state of MAC layer */
    lwmac_state_t state;
    /* Track if a transmission might have corrupted a received packet */
    bool rx_started;
    /* Own address */
    l2_addr_t l2_addr;
    lwmac_rx_t rx;
    lwmac_tx_t tx;
    /* Feedback of last packet that was sent */
    lwmac_tx_feedback_t tx_feedback;
    /* Store timeouts used for protocol */
    lwmac_timeout_t timeouts[LWMAC_TIMEOUT_COUNT];
    /* Used to calculate wakeup times */
    uint32_t last_wakeup;
    /* Keep track of duty cycling to avoid late RTT events after stopping */
    bool dutycycling_active;
    /* Used internally for rescheduling state machine update, e.g. after state
     * transition caused in update */
    bool needs_rescheduling;
} lwmac_t;

#define LWMAC_INIT { \
/* pid */                   KERNEL_PID_UNDEF,  \
/* netdev */                NULL, \
/* state */                 UNDEF, \
/* rx_in_progress */        false, \
/* l2_addr */               {}, \
/* rx */                    LWMAC_RX_INIT, \
/* tx */                    LWMAC_TX_INIT, \
/* tx_feedback */           LWMAC_TX_FEEDBACK_INIT, \
/* timeouts */              {}, \
/* last_wakeup */           0, \
/* dutycycling_active */    false, \
/* needs_rescheduling */    false \
}

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_TYPES_H_ */
/** @} */
