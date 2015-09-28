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
 * @brief       Implementation of the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "kernel.h"
#include "msg.h"
#include "thread.h"
#include "timex.h"
#include "priority_queue.h"
#include "periph/rtt.h"
#include "net/gnrc.h"
#include "net/gnrc/lwmac/lwmac.h"
#include "net/gnrc/lwmac/packet_queue.h"
#include "net/gnrc/lwmac/timeout.h"
#include "net/gnrc/lwmac/tx_state_machine.h"
#include "net/gnrc/lwmac/rx_state_machine.h"
#include "include/lwmac_internal.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define LOG_LEVEL LOG_INFO
#include "log.h"

#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_INFO
#undef LOG_DEBUG

#define LOG_ERROR(...) LOG(LOG_ERROR, "ERROR: [lwmac] " __VA_ARGS__)
#define LOG_WARNING(...) LOG(LOG_WARNING, "WARNING: [lwmac] " __VA_ARGS__)
#define LOG_INFO(...) LOG(LOG_INFO, "[lwmac] " __VA_ARGS__)
#define LOG_DEBUG(...) LOG(LOG_DEBUG, "[lwmac] " __VA_ARGS__)

/******************************************************************************/

/* Internal state of lwMAC */
static lwmac_t lwmac = LWMAC_INIT;

/******************************************************************************/

static bool lwmac_update(void);
static void lwmac_set_state(lwmac_state_t newstate);
static void lwmac_schedule_update(void);
static bool lwmac_needs_update(void);
static void rtt_handler(uint32_t event);

/******************************************************************************/

// TODO: Don't use global variables
inline void lwmac_schedule_update(void)
{
    lwmac.needs_rescheduling = true;
}

/******************************************************************************/

// TODO: Don't use global variables
inline bool lwmac_needs_update(void)
{
    return lwmac.needs_rescheduling;
}

/******************************************************************************/

// TODO: Don't use global variables
void lwmac_set_state(lwmac_state_t newstate)
{
    lwmac_state_t oldstate = lwmac.state;

    if(newstate == oldstate)
        return;

    if(newstate >= STATE_COUNT) {
        LOG_ERROR("Trying to set invalid state %u\n", newstate);
        return;
    }

    /* Already change state, but might be reverted to oldstate when needed */
    lwmac.state = newstate;

    /* Actions when leaving old state */
    switch(oldstate)
    {
    case RECEIVING:
    case TRANSMITTING:
        /* Enable duty cycling again */
        rtt_handler(LWMAC_EVENT_RTT_RESUME);
        break;

    default:
        break;
    }

    /* Actions when entering new state */
    switch(newstate)
    {
    /*********************** Operation states *********************************/
    case LISTENING:
        _set_netdev_state(&lwmac, NETOPT_STATE_IDLE);
        break;

    case SLEEPING:
        /* Put transceiver to sleep */
        _set_netdev_state(&lwmac, NETOPT_STATE_SLEEP);
        break;

    /* Trying to send data */
    case TRANSMITTING:
        rtt_handler(LWMAC_EVENT_RTT_PAUSE); /* No duty cycling while RXing */
        _set_netdev_state(&lwmac, NETOPT_STATE_IDLE);  /* Power up netdev */
        break;

    /* Receiving incoming data */
    case RECEIVING:
        rtt_handler(LWMAC_EVENT_RTT_PAUSE); /* No duty cycling while TXing */
        _set_netdev_state(&lwmac, NETOPT_STATE_IDLE);  /* Power up netdev */
        break;

    case STOPPED:
        _set_netdev_state(&lwmac, NETOPT_STATE_OFF);
        break;

    /*********************** Control states ***********************************/
    case START:
        rtt_handler(LWMAC_EVENT_RTT_START);
        lwmac_set_state(LISTENING);
        break;

    case STOP:
        rtt_handler(LWMAC_EVENT_RTT_STOP);
        lwmac_set_state(STOPPED);
        break;

    case RESET:
        LOG_WARNING("Reset not yet implemented\n");
        lwmac_set_state(STOP);
        lwmac_set_state(START);
        break;

    /**************************************************************************/
    default:
        LOG_DEBUG("No actions for entering state %u\n", newstate);
        return;
    }

    lwmac_schedule_update();
}

/******************************************************************************/

/* Main state machine. Call whenever something happens */
// TODO: Don't use global variables
bool lwmac_update(void)
{
    lwmac.needs_rescheduling = false;

    switch(lwmac.state)
    {
    case SLEEPING:

        /* If a packet is scheduled, no other (possible earlier) packet can be
         * sent before the first one is handled
         */
        if( !lwmac_timeout_is_running(&lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP) ) {

            /* Time in microseconds when the earliest (phase) destination node
             * wakes up that we have packets for. */
            int time_until_tx = _time_until_tx_us(&lwmac);

            if(time_until_tx > 0) {
                LOG_INFO("There's something to send!\n");

                /* If there's not enough time to prepare a WR to catch the phase
                 * postpone to next interval */
                if (time_until_tx < LWMAC_WR_PREPARATION_US) {
                    time_until_tx += LWMAC_WAKEUP_INTERVAL_MS * 1000;
                }

                time_until_tx -= LWMAC_WR_PREPARATION_US;

                timex_t interval = {0, time_until_tx};
                LOG_INFO("Schedule wakeup in: %"PRIu32" us\n", interval.microseconds);
                lwmac_set_timeout(&lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP, &interval);
            } else {
                /* LOG_DEBUG("Nothing to do, why did you get called?\n"); */
            }
        } else {
            if(lwmac_timeout_is_expired(&lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP)) {
                LOG_INFO("Got timeout for dest wakeup, ticks: %"PRIu32"\n", rtt_get_counter());
                lwmac_set_state(TRANSMITTING);
            } else {
                LOG_DEBUG("Waiting for TIMEOUT_WAIT_FOR_DEST_WAKEUP\n");
            }
        }
        break;

    case LISTENING:
        if(packet_queue_length(&lwmac.rx.queue) > 0) {
            lwmac_set_state(RECEIVING);
        }
        break;

    case RECEIVING:
    {
        char* rx_success = "";
        lwmac_rx_state_t state_rx = lwmac.rx.state;

        switch(state_rx)
        {
        case RX_STATE_STOPPED:
        {
            lwmac_rx_start(&lwmac);
            lwmac_rx_update(&lwmac);
            break;
        }
        case RX_STATE_FAILED:
            rx_success = "NOT ";
        case RX_STATE_SUCCESSFUL:
            LOG_INFO("Reception finished %ssuccessfully\n", rx_success);
            lwmac_rx_stop(&lwmac);
            lwmac_set_state(SLEEPING);
            break;
        default:
            lwmac_rx_update(&lwmac);
        }

        /* If state has changed, reschedule main state machine */
        if(state_rx != lwmac.rx.state)
        {
            lwmac_schedule_update();
        }
        break;
    }
    case TRANSMITTING:
    {
        char* tx_success = "";
        lwmac_tx_state_t state_tx = lwmac.tx.state;

        switch(state_tx)
        {
        case TX_STATE_STOPPED:
        {
            gnrc_pktsnip_t* pkt = NULL;

            int neighbour_id = _next_tx_neighbour(lwmac.tx.queues);
            lwmac_tx_queue_t* tx_queue = &(lwmac.tx.queues[neighbour_id]);

            if( (pkt = packet_queue_pop(&tx_queue->queue)) )
            {
                lwmac_tx_start(&lwmac, pkt, tx_queue);
                lwmac_tx_update(&lwmac);
            } else {
                LOG_WARNING("In 'TRANSMITTING' but tx.queue is empty\n");
                lwmac_set_state(SLEEPING);
            }
            break;
        }

        case TX_STATE_FAILED:
            tx_success = "NOT ";
        case TX_STATE_SUCCESSFUL:
            LOG_INFO("Transmission of pkt @ %p finished %ssuccessfully "
                     "(%"PRIu32" WRs)\n",
                     lwmac.tx.packet, tx_success, lwmac.tx.wr_sent);
            lwmac_tx_stop(&lwmac);
            lwmac_clear_timeout(&lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP);
            lwmac_set_state(SLEEPING);
            break;
        default:
            lwmac_tx_update(&lwmac);
        }

        /* If state has changed, reschedule main state machine */
        if(state_tx != lwmac.tx.state)
        {
            lwmac.needs_rescheduling = true;
        }
        break;
    }
    default:
        LOG_DEBUG("No actions in state %u\n", lwmac.state);
    }

    return lwmac.needs_rescheduling;
}

/******************************************************************************/

static void rtt_cb(void* arg)
{
    msg_t msg;
    msg.content.value = ((uint32_t) arg ) & 0xffff;
    msg.type = LWMAC_EVENT_RTT_TYPE;
    msg_send(&msg, lwmac.pid);
}

/******************************************************************************/

// TODO: Don't use global variables
void rtt_handler(uint32_t event)
{
    uint32_t alarm;
    switch(event & 0xffff)
    {
    case LWMAC_EVENT_RTT_WAKEUP_PENDING:
        lwmac.last_wakeup = rtt_get_alarm();
        alarm = _next_inphase_event(lwmac.last_wakeup, RTT_MS_TO_TICKS(LWMAC_WAKEUP_DURATION_MS));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_SLEEP_PENDING);
        lpm_prevent_sleep = 1;
        lwmac_set_state(LISTENING);
        break;

    case LWMAC_EVENT_RTT_SLEEP_PENDING:
        alarm = _next_inphase_event(lwmac.last_wakeup, RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_WAKEUP_PENDING);
        lwmac_set_state(SLEEPING);
        lpm_prevent_sleep = 0;
        break;

    /* Set initial wakeup alarm that starts the cycle */
    case LWMAC_EVENT_RTT_START:
        LOG_DEBUG("RTT: Initialize duty cycling\n");
        alarm = rtt_get_counter() + RTT_MS_TO_TICKS(150);
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_SLEEP_PENDING);
        lpm_prevent_sleep = 1;
        break;

    case LWMAC_EVENT_RTT_STOP:
    case LWMAC_EVENT_RTT_PAUSE:
        rtt_clear_alarm();
        LOG_DEBUG("RTT: Stop duty cycling, now in state %u\n", lwmac.state);
        lpm_prevent_sleep = 1;
        break;

    case LWMAC_EVENT_RTT_RESUME:
        LOG_DEBUG("RTT: Resume duty cycling\n");
        alarm = _next_inphase_event(lwmac.last_wakeup, RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_WAKEUP_PENDING);
        lpm_prevent_sleep = 0;
        break;
    }
}

/******************************************************************************/

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event         type of event
 * @param[in] data          optional parameter
 */
// TODO: Don't use global variables
static void _event_cb(gnrc_netdev_event_t event, void *data)
{
    switch(event)
    {
    case NETDEV_EVENT_RX_STARTED:
        LOG_DEBUG("NETDEV_EVENT_RX_STARTED\n");
        lwmac.rx_started = true;
        break;
    case NETDEV_EVENT_RX_COMPLETE:
    {
        LOG_DEBUG("NETDEV_EVENT_RX_COMPLETE\n");

        gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t *) data;

        /* Prevent packet corruption when a packet is sent before the previous
         * received packet has been downloaded. This happens e.g. when a timeout
         * expires that causes the tx state machine to send a packet. When a
         * packet arrives after the timeout, the notification is queued but the
         * tx state machine continues to send and then destroys the received
         * packet in the frame buffer. After completion, the queued notification
         * will be handled a corrupted packet will be downloaded. Therefore
         * keep track that RX_STARTED is followed by RX_COMPLETE.
         *
         * TODO: transceivers might have 2 frame buffers, so make this optional
         */
        if(!lwmac.rx_started) {
            LOG_WARNING("Maybe sending kicked in and frame buffer is now corrupted\n");
            gnrc_pktbuf_release(pkt);
            break;
        }

        lwmac.rx_started = false;

        if(!packet_queue_push(&lwmac.rx.queue, pkt, 0))
        {
            LOG_ERROR("Can't push RX packet @ %p, memory full?\n", pkt);
            gnrc_pktbuf_release(pkt);
            break;
        }
        lwmac_schedule_update();
        break;
    }
    case NETDEV_EVENT_TX_STARTED:
        lwmac.tx_feedback = TX_FEEDBACK_UNDEF;
        lwmac.rx_started = false;
        lwmac_schedule_update();
        LOG_INFO("EVT TX_STARTED\n");
        break;
    case NETDEV_EVENT_TX_COMPLETE:
        lwmac.tx_feedback = TX_FEEDBACK_SUCCESS;
        lwmac.rx_started = false;
        lwmac_schedule_update();
        break;
    case NETDEV_EVENT_TX_NOACK:
        LOG_DEBUG("NETDEV_EVENT_TX_NOACK\n");
        lwmac.tx_feedback = TX_FEEDBACK_NOACK;
        lwmac.rx_started = false;
        lwmac_schedule_update();
        break;
    case NETDEV_EVENT_TX_MEDIUM_BUSY:
        LOG_DEBUG("NETDEV_EVENT_TX_MEDIUM_BUSY\n");
        lwmac.tx_feedback = TX_FEEDBACK_BUSY;
        lwmac.rx_started = false;
        lwmac_schedule_update();
        break;

    default:
        LOG_WARNING("Unhandled netdev event: %u\n", event);
    }
}

/**
 * @brief   Startup code and event loop of the LWMAC layer
 *
 * @param[in] args          expects a pointer to the underlying netdev device
 *
 * @return                  never returns
 */
// TODO: Don't use global variables
static void *_lwmac_thread(void *args)
{
    gnrc_netdev_t* dev = lwmac.netdev = (gnrc_netdev_t *)args;
    gnrc_netapi_opt_t* opt;
    int res;
    msg_t msg, reply, msg_queue[LWMAC_IPC_MSG_QUEUE_SIZE];

    LOG_INFO("Starting lwMAC\n");

    /* RTT is used for scheduling wakeup */
    rtt_init();

    /* Store pid globally, so that IRQ can use it to send msg */
    lwmac.pid = thread_getpid();

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, LWMAC_IPC_MSG_QUEUE_SIZE);
    /* save the PID to the device descriptor and register the device */
    dev->mac_pid = lwmac.pid;
    gnrc_netif_add(lwmac.pid);
    /* register the event callback with the device driver */
    dev->driver->add_event_callback(dev, _event_cb);

    /* Enable RX- and TX-started interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    dev->driver->set(dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Enable preloading, so packet will only be sent when netdev state will be
     * set to NETOPT_STATE_TX */
    dev->driver->set(dev, NETOPT_PRELOADING, &enable, sizeof(enable));

    /* Get own address from netdev */
    lwmac.addr_len = dev->driver->get(dev, NETOPT_ADDRESS, &lwmac.addr, 8);
    if( (lwmac.addr_len < 0) || (lwmac.addr_len > 8) ) {
        LOG_ERROR("Couldn't aquire hw address from netdev\n");
        return NULL;
    }

    /* Start duty cycling */
    lwmac_set_state(START);

    /* start the event loop */
    while (1) {

        msg_receive(&msg);

        /* Handle NETDEV, NETAPI, RTT and TIMEOUT messages */
        switch (msg.type) {

        /* RTT raised an interrupt */
        case LWMAC_EVENT_RTT_TYPE:
            rtt_handler(msg.content.value);
            lwmac_schedule_update();
            break;

        /* An lwmac timeout occured */
        case LWMAC_EVENT_TIMEOUT_TYPE:
        {
            LOG_DEBUG("LWMAC_EVENT_VTIMER_TYPE\n");
            lwmac_timeout_t* timeout = (lwmac_timeout_t*) msg.content.ptr;
            timeout->expired = true;
            lwmac_schedule_update();
            break;
        }

        /* Transceiver raised an interrupt */
        case GNRC_NETDEV_MSG_TYPE_EVENT:
            LOG_DEBUG("GNRC_NETDEV_MSG_TYPE_EVENT received\n");
            /* Forward event back to driver */
            dev->driver->isr_event(dev, msg.content.value);
            break;

        /* TX: Queue for sending */
        case GNRC_NETAPI_MSG_TYPE_SND:
        {
            // TODO: how to announce failure to upper layers?

            LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SND received\n");
            gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) msg.content.ptr;

            _queue_tx_packet(&lwmac, pkt);

            lwmac_schedule_update();
            break;
        }
        /* NETAPI set/get. Can't this be refactored away from here? */
        case GNRC_NETAPI_MSG_TYPE_SET:
        {
            LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SET received\n");
            opt = (gnrc_netapi_opt_t *)msg.content.ptr;

            /* Depending on option forward to NETDEV or handle here */
            switch(opt->opt)
            {
            /* Handle state change requests */
            case NETOPT_STATE:
            {
                netopt_state_t* state = (netopt_state_t*) opt->data;
                res = opt->data_len;
                switch(*state)
                {
                case NETOPT_STATE_OFF:
                    lwmac_set_state(STOP);
                    break;
                case NETOPT_STATE_IDLE:
                    lwmac_set_state(START);
                    break;
                case NETOPT_STATE_RESET:
                    lwmac_set_state(RESET);
                    break;
                default:
                    res = -EINVAL;
                    LOG_ERROR("NETAPI tries to set unsupported state %u\n",
                          *state);
                }
                lwmac_schedule_update();
                break;
            }
            /* Forward to netdev by default*/
            default:
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                LOG_DEBUG("Response of netdev->set: %i\n", res);
            }

            /* send reply to calling thread */
            reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
            reply.content.value = (uint32_t)res;
            msg_reply(&msg, &reply);
            break;
        }
        case GNRC_NETAPI_MSG_TYPE_GET:
            /* TODO: filter out MAC layer options -> for now forward
                     everything to the device driver */
            LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_GET received\n");
            /* read incoming options */
            opt = (gnrc_netapi_opt_t *)msg.content.ptr;
            /* get option from device driver */
            res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
            LOG_DEBUG("Response of netdev->get: %i\n", res);
            /* send reply to calling thread */
            reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
            reply.content.value = (uint32_t)res;
            msg_reply(&msg, &reply);
            break;

        default:
            LOG_ERROR("Unknown command %" PRIu16 "\n", msg.type);
            break;
        }

        /* Execute main state machine because something just happend*/
        while(lwmac_needs_update()) {
            lwmac_update();
        }
    }

    LOG_ERROR("terminated\n");

    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_lwmac_init(char *stack, int stacksize, char priority,
                        const char *name, gnrc_netdev_t *dev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (dev == NULL || dev->driver == NULL) {
        LOG_ERROR("No netdev supplied or driver not set\n");
        return -ENODEV;
    }

    /* Prevent sleeping until first RTT alarm is set */
    lpm_prevent_sleep = 1;

    /* create new LWMAC thread */
    res = thread_create(stack, stacksize, priority, CREATE_STACKTEST,
                         _lwmac_thread, (void *)dev, name);
    if (res <= 0) {
        LOG_ERROR("Couldn't create thread\n");
        lpm_prevent_sleep = 0;
        return -EINVAL;
    }

    return res;
}
