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
 * @brief       Implementation of TX state machine
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include "net/gnrc.h"
#include "net/gnrc/lwmac/lwmac.h"
#include "net/gnrc/lwmac/tx_state_machine.h"
#include "net/gnrc/lwmac/timeout.h"
#include "net/gnrc/lwmac/packet_queue.h"


#define ENABLE_DEBUG    (1)
#include "debug.h"

#define LOG_LEVEL LOG_INFO
#include "log.h"

#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_INFO
#undef LOG_DEBUG

#define LOG_ERROR(...) LOG(LOG_ERROR, "ERROR: [lwmac-tx] " __VA_ARGS__)
#define LOG_WARNING(...) LOG(LOG_WARNING, "WARNING: [lwmac-tx] " __VA_ARGS__)
#define LOG_INFO(...) LOG(LOG_INFO, "[lwmac-tx] " __VA_ARGS__)
#define LOG_DEBUG(...) LOG(LOG_DEBUG, "[lwmac-tx] " __VA_ARGS__)

/* Break out of switch and mark the need for rescheduling */
#define GOTO_TX_STATE(tx_state, do_resched) lwmac->tx.state = tx_state; \
                                reschedule = do_resched; \
                                break

/******************************************************************************/

void lwmac_tx_start(lwmac_t* lwmac, gnrc_pktsnip_t* pkt)
{
    assert(lwmac != NULL);
    assert(pkt != NULL);

    if(lwmac->tx.packet) {
        LOG_WARNING("Starting but tx.packet is still set\n");
        gnrc_pktbuf_release(lwmac->tx.packet);
    }

    lwmac->tx.packet = pkt;
    lwmac->tx.state = TX_STATE_INIT;
    lwmac->tx.wr_sent = 0;
}

/******************************************************************************/

void lwmac_tx_stop(lwmac_t* lwmac)
{
    if(!lwmac)
        return;

    lwmac_reset_timeouts(lwmac);
    lwmac->tx.state = TX_STATE_STOPPED;

    /* Release packet in case of failure */
    gnrc_pktbuf_release(lwmac->tx.packet);
    lwmac->tx.packet = NULL;
}

/******************************************************************************/

/* Returns whether rescheduling is needed or not */
static bool _lwmac_tx_update(lwmac_t* lwmac)
{
    bool reschedule = false;

    if(!lwmac)
        return reschedule;

    switch(lwmac->tx.state)
    {
    case TX_STATE_INIT:
    {
        lwmac_reset_timeouts(lwmac);

        /* 150% of wakeup interval */
        timex_t interval = {0, LWMAC_WAKEUP_INTERVAL_MS * 1500};
        lwmac_set_timeout(lwmac, TIMEOUT_NO_RESPONSE, &interval);

        GOTO_TX_STATE(TX_STATE_SEND_WR, true);
    }
    case TX_STATE_SEND_WR:
    {
        LOG_DEBUG("TX_STATE_SEND_WR\n");

        gnrc_pktsnip_t* pkt;
        gnrc_netif_hdr_t *nethdr;
        uint8_t* dst_addr = NULL;
        int addr_len;

        /* Set timeout for next WR in case no WA will be received */
        timex_t interval = {0, LWMAC_TIME_BETWEEN_WR_US};
        lwmac_set_timeout(lwmac, TIMEOUT_WR, &interval);

        /* Get destination address */
        addr_len = _get_dest_address(lwmac->tx.packet, &dst_addr);
        if(addr_len <= 0 || addr_len > 8) {
            LOG_ERROR("Invalid address length: %i\n", addr_len);
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* Assemble WR */
        lwmac_hdr_t lwmac_hdr = {FRAMETYPE_WR, false};

        pkt = gnrc_pktbuf_add(NULL, &lwmac_hdr, sizeof(lwmac_hdr), GNRC_NETTYPE_LWMAC);
        if(pkt == NULL) {
            LOG_ERROR("Cannot allocate pktbuf of type GNRC_NETTYPE_LWMAC\n");
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t) + addr_len, GNRC_NETTYPE_NETIF);
        if(pkt == NULL) {
            LOG_ERROR("Cannot allocate pktbuf of type GNRC_NETTYPE_NETIF\n");
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* We wouldn't get here if add the NETIF header had failed, so no
           sanity checks needed */
        nethdr = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

        /* Construct NETIF header and insert address for WR packet */
        gnrc_netif_hdr_init(nethdr, 0, addr_len);
        gnrc_netif_hdr_set_dst_addr(nethdr, dst_addr, addr_len);

        /* Disable Auto ACK */
        netopt_enable_t autoack = NETOPT_DISABLE;
        lwmac->netdev->driver->set(lwmac->netdev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

        if(_get_netdev_state(lwmac) == NETOPT_STATE_RX) {
            LOG_WARNING("Receiving now, so cancel sending WR\n");
            gnrc_pktbuf_release(pkt);
            GOTO_TX_STATE(TX_STATE_WAIT_FOR_WA, false);
        }

        /* Send WR */
        lwmac->netdev->driver->send_data(lwmac->netdev, pkt);

        /* Flush RX queue, TODO: maybe find a way without loosing RX packets */
        packet_queue_flush(&lwmac->rx.queue);

        lwmac->tx.wr_sent++;

        GOTO_TX_STATE(TX_STATE_WAIT_FOR_WA, false);
    }
    case TX_STATE_WAIT_FOR_WA:
    {
        LOG_DEBUG("TX_STATE_WAIT_FOR_WA\n");

        gnrc_pktsnip_t* pkt;
        bool found_wa = false;

        if(lwmac_timeout_is_expired(lwmac, TIMEOUT_WR)) {
            GOTO_TX_STATE(TX_STATE_SEND_WR, true);
        }

        if(lwmac_timeout_is_expired(lwmac, TIMEOUT_NO_RESPONSE)) {
            LOG_DEBUG("No response from destination\n");
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        if(_get_netdev_state(lwmac) == NETOPT_STATE_RX) {
            LOG_WARNING("Wait for completion of frame reception\n");
            break;
        }

        while( (pkt = packet_queue_pop(&lwmac->rx.queue)) != NULL )
        {
            LOG_DEBUG("Inspecting pkt @ %p\n", pkt);

            /* Dissect lwMAC header */
            gnrc_pktbuf_mark(pkt, sizeof(lwmac_hdr_t), GNRC_NETTYPE_LWMAC);

            if(_accept_packet(pkt, FRAMETYPE_WA, lwmac)) {
                LOG_DEBUG("Found WA\n");
                lwmac_clear_timeout(lwmac, TIMEOUT_WR);
                lwmac_clear_timeout(lwmac, TIMEOUT_NO_RESPONSE);
                found_wa = true;
                break;
            }

            /* Cleanup packet that was popped and discarded */
            LOG_DEBUG("Reject pkt @ %p\n", pkt);
            gnrc_pktbuf_release(pkt);
        }

        if(!found_wa) {
            LOG_DEBUG("No WA yet\n");
            break;
        }

        /* We don't need the packet anymore */
        gnrc_pktbuf_release(pkt);

        /* We've got our WA, so discard the rest, TODO: no flushing */
        packet_queue_flush(&lwmac->rx.queue);

        GOTO_TX_STATE(TX_STATE_SEND_DATA, true);
    }
    case TX_STATE_SEND_DATA:
    {
        LOG_DEBUG("TX_STATE_SEND_DATA\n");

        gnrc_pktsnip_t* pkt = lwmac->tx.packet;

        /* Enable Auto ACK again */
        netopt_enable_t autoack = NETOPT_ENABLE;
        lwmac->netdev->driver->set(lwmac->netdev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

        /* Insert lwMAC header above NETIF header */
        lwmac_hdr_t hdr = {FRAMETYPE_DATA, false};
        pkt->next = gnrc_pktbuf_add(pkt->next, &hdr, sizeof(hdr), GNRC_NETTYPE_LWMAC);

        /* Send data */
        lwmac->netdev->driver->send_data(lwmac->netdev, pkt);

        /* Packet has been released by netdev, so drop pointer */
        lwmac->tx.packet = NULL;

        GOTO_TX_STATE(TX_STATE_WAIT_FEEDBACK, false);
    }
    case TX_STATE_WAIT_FEEDBACK:
    {
        LOG_DEBUG("TX_STATE_WAIT_FEEDBACK\n");
        if(lwmac->tx_feedback == TX_FEEDBACK_UNDEF) {
            break;
        } else if(lwmac->tx_feedback == TX_FEEDBACK_SUCCESS) {
            GOTO_TX_STATE(TX_STATE_SUCCESSFUL, true);
        } else if(lwmac->tx_feedback == TX_FEEDBACK_NOACK) {
            LOG_ERROR("Not ACKED\n");
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        } else if(lwmac->tx_feedback == TX_FEEDBACK_BUSY) {
            LOG_ERROR("Channel busy \n");
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        LOG_ERROR("Tx feedback unhandled: %i\n", lwmac->tx_feedback);
        GOTO_TX_STATE(TX_STATE_FAILED, true);
    }
    case TX_STATE_SUCCESSFUL:
    case TX_STATE_FAILED:
        lwmac_reset_timeouts(lwmac);
        break;
    case TX_STATE_STOPPED:
        LOG_DEBUG("Transmission state machine is stopped\n");
    }

    return reschedule;
}

/******************************************************************************/

void lwmac_tx_update(lwmac_t* lwmac)
{
    /* Update until no rescheduling needed */
    while(_lwmac_tx_update(lwmac));
}

/******************************************************************************/
