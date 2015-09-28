/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_lwmac
 * @file
 * @brief       Internal functions of LWMAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include "periph/rtt.h"
#include "priority_queue.h"
#include "net/gnrc.h"
#include "net/gnrc/lwmac/lwmac.h"
#include "net/gnrc/lwmac/packet_queue.h"
#include "net/gnrc/lwmac/timeout.h"
#include "include/lwmac_internal.h"


#define ENABLE_DEBUG    (0)
#include "debug.h"

/******************************************************************************/

int _get_dest_address(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[])
{
    int res;
    gnrc_netif_hdr_t* netif_hdr;

    if(!pkt)
        return -ENODEV;

    netif_hdr = (gnrc_netif_hdr_t*) pkt->data;
    if( (res = netif_hdr->dst_l2addr_len) <= 0)
        return -ENOENT;

    *pointer_to_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
    return res;
}

/******************************************************************************/

/* Find a payload based on it's protocol type */
void* _gnrc_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type)
{
    while(pkt != NULL)
    {
        if(pkt->type == type) {
            return pkt->data;
        }
        pkt = pkt->next;
    }
    return NULL;
}

/******************************************************************************/

int _find_neighbour_queue(lwmac_t* lwmac, uint8_t* dst_addr, int addr_len)
{
    lwmac_tx_queue_t* queues = lwmac->tx.queues;

    for(int i = 0; i < LWMAC_NEIGHBOUR_COUNT; i++) {
        if(queues[i].addr_len == addr_len) {
            if(memcmp(&(queues[i].addr), dst_addr, addr_len) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/******************************************************************************/

/* Free first empty queue that is not active */
int _free_neighbour_queue(lwmac_t* lwmac)
{
    lwmac_tx_queue_t* queues = lwmac->tx.queues;

    for(int i = 0; i < LWMAC_NEIGHBOUR_COUNT; i++) {
        if( (packet_queue_length(&(queues[i].queue)) == 0) &&
            (&queues[i] != lwmac->tx.current_queue) ) {
            /* Mark as free */
            queues[i].addr_len = 0;
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

int _alloc_neighbour_queue(lwmac_t* lwmac)
{
    lwmac_tx_queue_t* queues = lwmac->tx.queues;

    for(int i = 0; i < LWMAC_NEIGHBOUR_COUNT; i++) {
        if(queues[i].addr_len == 0) {
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

//static bool _tx_packet_present(lwmac_t* lwmac)
//{
//    for(int i = 0; i < LWMAC_NEIGHBOUR_COUNT; i++) {
//        if( (lwmac->tx.queues[i].addr_len > 0) &&
//            (packet_queue_length(&lwmac->tx.queues[i].queue) > 0) ) {
//            return true;
//        }
//    }
//    return false;
//}

/******************************************************************************/
/* TODO: maybe static inline */
uint32_t _ticks_to_phase(uint32_t ticks)
{
    return (ticks % RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS));
}

/******************************************************************************/
/* TODO: maybe static inline */
uint32_t _phase_to_ticks(uint32_t phase)
{
    uint32_t rtt_now = rtt_get_counter();
    uint32_t phase_now = _ticks_to_phase(rtt_now);

    /* Start of current interval */
    rtt_now -= phase_now;

    /* Phase only in next interval */
    if(phase < phase_now) {
        rtt_now += RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS);
    }

    /* Advance to phase */
    return (rtt_now + phase);
}

/******************************************************************************/
/* TODO: maybe static inline */
uint32_t _phase_now(void)
{
    return _ticks_to_phase(rtt_get_counter());
}

/******************************************************************************/
/* TODO: maybe static inline */
uint32_t _ticks_until_phase(uint32_t phase)
{
    long int tmp = phase - _phase_now();
    if(tmp < 0) {
        /* Phase in next interval */
        tmp += RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS);
    }

    return (uint32_t)tmp;
}

/******************************************************************************/

/* Find the neighbour that has a packet queued and is next for sending */
int _next_tx_neighbour(lwmac_tx_queue_t queues[])
{
    int next = -1;

    uint32_t phase_check;
    uint32_t phase_nearest = LWMAC_PHASE_MAX;

    for(int i = 0; i < LWMAC_NEIGHBOUR_COUNT; i++) {

        if(packet_queue_length(&(queues[i].queue)) > 0) {

            /* Unknown destinations are initialized with their phase at the end
             * of the local interval, so known destinations that still wakeup
             * in this interval will be preferred. */
            phase_check = _ticks_until_phase(queues[i].phase);

            if(phase_check <= phase_nearest) {
                next = i;
                phase_nearest = phase_check;
                DEBUG("[lwmac-int] Advancing queue #%d\n", i);
            }
        }
    }

    return next;
}

/******************************************************************************/

int _time_until_tx_us(lwmac_t* lwmac)
{
    int neighbour_id = _next_tx_neighbour(lwmac->tx.queues);

    if(neighbour_id < 0) {
        return -1;
    }
    uint32_t neighbour_phase = lwmac->tx.queues[neighbour_id].phase;
    return RTT_TICKS_TO_US(_ticks_until_phase(neighbour_phase));
}

/******************************************************************************/

bool _queue_tx_packet(lwmac_t* lwmac,  gnrc_pktsnip_t* pkt)
{
    uint8_t* addr;
    int addr_len;
    int neighbour_id;
    bool queue_existed = true;

    lwmac_tx_queue_t* queues = lwmac->tx.queues;

    /* Get destination address of packet */
    addr_len = _get_dest_address(pkt, &addr);
    if(addr_len <= 0) {
        DEBUG("[lwmac-int] Packet has no destination address\n");
        gnrc_pktbuf_release(pkt);
        return false;
    }

    /* Search for existing queue for destination */
    neighbour_id = _find_neighbour_queue(lwmac, addr, addr_len);

    /* Neighbour node doesn't have a queue yet */
    if(neighbour_id < 0) {
        /* Try to allocate queue */
        neighbour_id = _alloc_neighbour_queue(lwmac);

        queue_existed = false;

        /* No queues left */
        if(neighbour_id < 0) {
            /* Try to free an unused queue */
            neighbour_id = _free_neighbour_queue(lwmac);

            /* All queues are in use, so reject */
            if(neighbour_id < 0) {
                DEBUG("[lwmac-int] Couldn't allocate tx queue for packet\n");
                gnrc_pktbuf_release(pkt);
                return false;
            }
        }
    }

    if(packet_queue_push(&(queues[neighbour_id].queue), pkt, 0) == NULL) {
        DEBUG("[lwmac-int] Can't push to tx queue, no entries left\n");
        gnrc_pktbuf_release(pkt);
        return false;
    }

    DEBUG("[lwmac-int] Queuing pkt to q #%d\n", neighbour_id);

    if(!queue_existed) {
        /* Setup new queue */
        queues[neighbour_id].addr_len = addr_len;
        queues[neighbour_id].phase = LWMAC_PHASE_UNINITIALIZED;
        memcpy(&(queues[neighbour_id].addr), addr, addr_len);
    }

    return true;
}

/******************************************************************************/

bool _accept_packet(gnrc_pktsnip_t* pkt, lwmac_frame_type_t expected_type, lwmac_t* lwmac)
{
    gnrc_netif_hdr_t* netif_hdr;
    lwmac_hdr_t* lwmac_hdr;
    uint8_t* own_addr = (uint8_t*) &(lwmac->addr);
    uint8_t* dst_addr;

    netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    if(netif_hdr == NULL) {
        DEBUG("[lwmac-int] Reject packet: no NETIF header\n");
        return false;
    }

    lwmac_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_LWMAC);
    if(lwmac_hdr == NULL) {
        DEBUG("[lwmac-int] Reject packet: no LWMAC header\n");
        return false;
    }

    if(lwmac_hdr->type != expected_type) {
        DEBUG("[lwmac-int] Reject packet: not of expected type, got 0x%x\n", lwmac_hdr->type);
        return false;
    }

    if(netif_hdr->dst_l2addr_len != lwmac->addr_len) {
        DEBUG("[lwmac-int] Reject packet: address length mismatch\n");
        return false;
    }

    /* TODO: detect broadcast */
    dst_addr =  gnrc_netif_hdr_get_dst_addr(netif_hdr);
    if(memcmp(own_addr,dst_addr, lwmac->addr_len) != 0) {
        DEBUG("[lwmac-int] Reject packet: not destined to this node\n");
        DEBUG("[lwmac-int] Own addr: 0x%x%x, Dest addr: 0x%x%x\n",
                  own_addr[0], own_addr[1],
                  dst_addr[0], dst_addr[1]);
        return false;
    }

    return true;
}

/******************************************************************************/

// TODO: Don't use global variables
void _set_netdev_state(lwmac_t* lwmac, netopt_state_t devstate)
{
    lwmac->netdev->driver->set(lwmac->netdev,
                               NETOPT_STATE,
                               &devstate,
                               sizeof(devstate));
}

/******************************************************************************/

netopt_state_t _get_netdev_state(lwmac_t* lwmac)
{
    netopt_state_t state;
    if (0 < lwmac->netdev->driver->get(lwmac->netdev,
                                       NETOPT_STATE,
                                       &state,
                                       sizeof(state))) {
        return state;
    }
    return -1;
}

/******************************************************************************/

/* Parameters in rtt timer ticks */
uint32_t _next_inphase_event(uint32_t last, uint32_t interval)
{
    uint32_t counter = rtt_get_counter();

    /* Counter did overflow since last wakeup */
    if(counter < last)
    {
        /* TODO: Not sure if this was tested :) */
        uint32_t tmp = -last;
        tmp /= interval;
        tmp++;
        last += tmp * interval;
    }

    /* Add margin to next wakeup so that it will be at least 2ms in the future */
    while(last < (counter + LWMAC_RTT_EVENT_MARGIN_TICKS))
    {
        last += interval;
    }

    return last;
}
