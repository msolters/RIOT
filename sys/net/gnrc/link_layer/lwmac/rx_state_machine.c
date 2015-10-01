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
 * @brief       Implementation of RX state machine
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include "net/gnrc.h"
#include "net/gnrc/lwmac/lwmac.h"
#include "net/gnrc/lwmac/packet_queue.h"
#include "include/rx_state_machine.h"
#include "include/timeout.h"
#include "include/lwmac_internal.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define LOG_LEVEL LOG_INFO
#include "log.h"

#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_INFO
#undef LOG_DEBUG

#define LOG_ERROR(...) LOG(LOG_ERROR, "ERROR: [lwmac-rx] " __VA_ARGS__)
#define LOG_WARNING(...) LOG(LOG_WARNING, "WARNING: [lwmac-rx] " __VA_ARGS__)
#define LOG_INFO(...) LOG(LOG_INFO, "[lwmac-rx] " __VA_ARGS__)
#define LOG_DEBUG(...) LOG(LOG_DEBUG, "[lwmac-rx] " __VA_ARGS__)

/* Break out of switch and mark the need for rescheduling */
#define GOTO_RX_STATE(rx_state, do_resched) lwmac->rx.state = rx_state; \
                                reschedule = do_resched; \
                                break

/******************************************************************************/

void lwmac_rx_start(lwmac_t* lwmac)
{
    if(lwmac == NULL)
        return;

    if(lwmac->rx.packet) {
        LOG_ERROR("Starting but pkt is still set\n");
        gnrc_pktbuf_release(lwmac->rx.packet);
    }

    lwmac->rx.state = RX_STATE_INIT;
    lwmac->rx.packet = NULL;
}

/******************************************************************************/

void lwmac_rx_stop(lwmac_t* lwmac)
{
    if(!lwmac)
        return;

    lwmac_reset_timeouts(lwmac);
    lwmac->rx.state = RX_STATE_STOPPED;
    /* release WR in case something went wrong */
    gnrc_pktbuf_release(lwmac->rx.packet);
    lwmac->rx.packet = NULL;
}

/******************************************************************************/

/* Returns whether rescheduling is needed or not */
static bool _lwmac_rx_update(lwmac_t* lwmac)
{
    bool reschedule = false;

    if(!lwmac)
        return reschedule;

    switch(lwmac->rx.state)
    {
    case RX_STATE_INIT:
        lwmac_reset_timeouts(lwmac);
        GOTO_RX_STATE(RX_STATE_WAIT_FOR_WR, true);

    case RX_STATE_WAIT_FOR_WR:
    {
        LOG_DEBUG("RX_STATE_WAIT_FOR_WR\n");

        gnrc_pktsnip_t* pkt;
        bool found_wr = false;

        while( (pkt = packet_queue_pop(&lwmac->rx.queue)) != NULL ) {
            LOG_DEBUG("Inspecting pkt @ %p\n", pkt);

            /* Dissect lwMAC header */
            gnrc_pktbuf_mark(pkt, sizeof(lwmac_hdr_t), GNRC_NETTYPE_LWMAC);

            if(_accept_packet(pkt, FRAMETYPE_WR, lwmac)) {
                found_wr = true;
                break;
            }

            /* Cleanup packet that was popped and discarded */
            LOG_DEBUG("Reject pkt @ %p\n", pkt);
            gnrc_pktbuf_release(pkt);
        }

        if(!found_wr) {
            LOG_DEBUG("No WR found\n");
            GOTO_RX_STATE(RX_STATE_FAILED, true);
        }

        /* Save WR so that we can extract the address later */
        lwmac->rx.packet = pkt;

        // TODO: don't flush queue
        packet_queue_flush(&lwmac->rx.queue);

        GOTO_RX_STATE(RX_STATE_SEND_WA, true);
    }
    case RX_STATE_SEND_WA:
    {
        LOG_DEBUG("RX_STATE_SEND_WA\n");

        gnrc_pktsnip_t* pkt;
        gnrc_netif_hdr_t* nethdr_wa;
        gnrc_netif_hdr_t* nethdr_wr;
        int addr_len;

        /* Extract NETIF header of WR packet */
        nethdr_wr = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(lwmac->rx.packet, GNRC_NETTYPE_NETIF);
        if(nethdr_wr == NULL) {
            LOG_ERROR("Couldn't find NETIF header inside WR\n");
            GOTO_RX_STATE(RX_STATE_FAILED, true);
        }

        /* Find out address length of destination */
        addr_len = nethdr_wr->src_l2addr_len;
        if(addr_len <= 0 || addr_len > 8) {
            LOG_ERROR("Invalid address length: %i\n", addr_len);
            GOTO_RX_STATE(RX_STATE_FAILED, true);
        }

        /* Assemble WA packet */
        lwmac_hdr_t lwmac_hdr = {FRAMETYPE_WA, false};

        pkt = gnrc_pktbuf_add(NULL, &lwmac_hdr, sizeof(lwmac_hdr), GNRC_NETTYPE_LWMAC);
        if(pkt == NULL) {
            LOG_ERROR("Cannot allocate pktbuf of type GNRC_NETTYPE_LWMAC\n");
            GOTO_RX_STATE(RX_STATE_FAILED, true);
        }

        pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t) + addr_len, GNRC_NETTYPE_NETIF);
        if(pkt == NULL) {
            LOG_ERROR("Cannot allocate pktbuf of type GNRC_NETTYPE_NETIF\n");
            GOTO_RX_STATE(RX_STATE_FAILED, true);
        }

        /* We wouldn't get here if add the NETIF header had failed, so no
           sanity checks needed */
        nethdr_wa = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

        /* Construct NETIF header and insert address for WA packet */
        gnrc_netif_hdr_init(nethdr_wa, 0, nethdr_wr->src_l2addr_len);
        gnrc_netif_hdr_set_dst_addr(nethdr_wa, gnrc_netif_hdr_get_src_addr(nethdr_wr), nethdr_wr->src_l2addr_len);

        /* Disable Auto ACK */
        netopt_enable_t autoack = NETOPT_DISABLE;
        lwmac->netdev->driver->set(lwmac->netdev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

        if(_get_netdev_state(lwmac) == NETOPT_STATE_RX) {
            LOG_WARNING("Receiving now, so cancel sending WA\n");
            gnrc_pktbuf_release(pkt);
            GOTO_RX_STATE(RX_STATE_WAIT_FOR_DATA, false);
            break;
        }

        /* Send WA */
        lwmac->netdev->driver->send_data(lwmac->netdev, pkt);
        _set_netdev_state(lwmac, NETOPT_STATE_TX);

        /* Release WR and WA packets */
        gnrc_pktbuf_release(lwmac->rx.packet);
        lwmac->rx.packet = NULL;

        /* Enable Auto ACK again for data reception */
        autoack = NETOPT_ENABLE;
        lwmac->netdev->driver->set(lwmac->netdev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

        GOTO_RX_STATE(RX_STATE_WAIT_WA_SENT, false);
    }
    case RX_STATE_WAIT_WA_SENT:
    {
        LOG_DEBUG("RX_STATE_WAIT_WA_SENT\n");

        if(lwmac->tx_feedback == TX_FEEDBACK_UNDEF) {
            LOG_DEBUG("WA not yet completely sent\n");
            break;
        }

        /* TODO: Maybe check if sending was successful? */

        /* Set timeout for expected data arrival */
        timex_t interval = {0, LWMAC_DATA_DELAY_US};
        lwmac_set_timeout(lwmac, TIMEOUT_DATA, &interval);

        GOTO_RX_STATE(RX_STATE_WAIT_FOR_DATA, false);
    }
    case RX_STATE_WAIT_FOR_DATA:
    {
        LOG_DEBUG("RX_STATE_WAIT_FOR_DATA\n");

        gnrc_pktsnip_t* pkt;
        bool found_data = false;
        bool found_wr = false;

        while( (pkt = packet_queue_pop(&lwmac->rx.queue)) != NULL ) {
            LOG_DEBUG("Inspecting pkt @ %p\n", pkt);

            /* Dissect lwMAC header */
            gnrc_pktbuf_mark(pkt, sizeof(lwmac_hdr_t), GNRC_NETTYPE_LWMAC);

            if(_accept_packet(pkt, FRAMETYPE_DATA, lwmac)) {
                LOG_DEBUG("Found DATA!\n");
                lwmac_clear_timeout(lwmac, TIMEOUT_DATA);
                found_data = true;
                break;
            } else if(_accept_packet(pkt, FRAMETYPE_WR, lwmac)) {
                /* Sender maybe didn't get the WA */
                LOG_WARNING("Found a WR while waiting for DATA\n");
                lwmac_clear_timeout(lwmac, TIMEOUT_DATA);
                found_wr = true;
                break;
            }

            gnrc_pktbuf_release(pkt);
        }

        /* If WA got lost we wait for data but we will be hammered with WR
         * packets. So a WR indicates a lost WA => reset RX state machine
         */
        if(found_wr) {
            LOG_INFO("WA probably got lost, reset RX state machine\n");
            /* Push WR back to rx queue */
            packet_queue_push(&lwmac->rx.queue, pkt, 0);
            /* Start over again */
            GOTO_RX_STATE(RX_STATE_INIT, true);
        }

        /* Only timeout if no packet (presumably the expected data) is being
         * received. This won't be blocked by WRs as they restart the state
         * machine (see above). */
        if( (lwmac_timeout_is_expired(lwmac, TIMEOUT_DATA)) &&
            (!lwmac->rx_started) ) {
            LOG_ERROR("DATA timed out\n");
            GOTO_RX_STATE(RX_STATE_FAILED, true);
        }

        if(!found_data) {
            LOG_WARNING("No DATA yet\n");
            break;
        }

        if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
            LOG_INFO("Unable to forward packet of type %i\n", pkt->type);
            gnrc_pktbuf_release(pkt);
        }

        GOTO_RX_STATE(RX_STATE_SUCCESSFUL, true);
    }
    case RX_STATE_SUCCESSFUL:
    case RX_STATE_FAILED:
        lwmac_reset_timeouts(lwmac);
        break;
    case RX_STATE_STOPPED:
        LOG_DEBUG("Reception state machine is stopped\n");
    }

    return reschedule;
}

/******************************************************************************/

void lwmac_rx_update(lwmac_t* lwmac)
{
    /* Update until no rescheduling needed */
    while(_lwmac_rx_update(lwmac));
}

/******************************************************************************/
