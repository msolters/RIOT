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

/* Find the neighbour that has a packet queued and is next for sending */
static int _next_tx_neighbour(lwmac_tx_queue_t queues[])
{
    /* For now just get the first packet that can be found */
    for(int i = 0; i < LWMAC_NEIGHBOUR_COUNT; i++) {
        if(packet_queue_length(&(queues[i].queue))) {
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

static bool _queue_tx_packet(lwmac_t* lwmac,  gnrc_pktsnip_t* pkt)
{
    uint8_t* addr;
    int addr_len;
    int neighbour_id;
    bool queue_existed = true;

    lwmac_tx_queue_t* queues = lwmac->tx.queues;

    /* Get destination address of packet */
    addr_len = _get_dest_address(pkt, &addr);
    if(addr_len <= 0) {
        LOG_ERROR("Packet has no destination address\n");
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
                LOG_ERROR("Couldn't allocate tx queue for packet\n");
                gnrc_pktbuf_release(pkt);
                return false;
            }
        }
    }

    if(packet_queue_push(&(queues[neighbour_id].queue), pkt, 0) == NULL) {
        LOG_ERROR("Can't push to tx queue, no entries left\n");
        gnrc_pktbuf_release(pkt);
        return false;
    }

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
        LOG_WARNING("Reject packet: no NETIF header\n");
        return false;
    }

    lwmac_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_LWMAC);
    if(lwmac_hdr == NULL) {
        LOG_WARNING("Reject packet: no LWMAC header\n");
        return false;
    }

    if(lwmac_hdr->type != expected_type) {
        LOG_WARNING("Reject packet: not of expected type, got 0x%x\n", lwmac_hdr->type);
        return false;
    }

    if(netif_hdr->dst_l2addr_len != lwmac->addr_len) {
        LOG_WARNING("Reject packet: address length mismatch\n");
        return false;
    }

    /* TODO: detect broadcast */
    dst_addr =  gnrc_netif_hdr_get_dst_addr(netif_hdr);
    if(memcmp(own_addr,dst_addr, lwmac->addr_len) != 0) {
        LOG_WARNING("Reject packet: not destined to this node\n");
        LOG_WARNING("Own addr: 0x%x%x, Dest addr: 0x%x%x\n",
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
    return NETOPT_STATE_RESET;
}

/******************************************************************************/

#define RTT_INPHASE_MARGIN_MS (2)
// TODO: Don't use global variables
/* Parameters in rtt timer ticks */
static uint32_t next_inphase_event(uint32_t last, uint32_t interval)
{
    uint32_t counter = rtt_get_counter();
//    uint32_t wakeup_interval = RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS);

    /* Counter did overflow since last wakeup */
    if(counter < last)
    {
        uint32_t tmp = -last;
        tmp /= interval;
        tmp++;
        last += tmp * interval;
    }

    /* Add margin to next wakeup so that it will be at least 2ms in the future */
    while(last < (counter + RTT_MS_TO_TICKS(RTT_INPHASE_MARGIN_MS)))
    {
        last += interval;
    }

    return last;
}

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

    LOG_DEBUG("State transition from %u to %u\n", oldstate, newstate);

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
        if(_next_tx_neighbour(lwmac.tx.queues) >= 0) {
            lwmac_set_state(TRANSMITTING);
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
            lwmac.needs_rescheduling = true;
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
        alarm = next_inphase_event(lwmac.last_wakeup, RTT_MS_TO_TICKS(LWMAC_WAKEUP_DURATION_MS));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_SLEEP_PENDING);
        LOG_DEBUG("RTT: Wakeup, set alarm=%"PRIu32", counter=%"PRIu32"\n", alarm, rtt_get_counter());
        lpm_prevent_sleep = 1;
        lwmac_set_state(LISTENING);
        break;

    case LWMAC_EVENT_RTT_SLEEP_PENDING:
        alarm = next_inphase_event(lwmac.last_wakeup, RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_WAKEUP_PENDING);
        LOG_DEBUG("RTT: Sleeping now, set alarm=%"PRIu32", counter=%"PRIu32"\n", alarm, rtt_get_counter());
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
        alarm = next_inphase_event(lwmac.last_wakeup, RTT_MS_TO_TICKS(LWMAC_WAKEUP_INTERVAL_MS));
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
    netopt_enable_t tell_start = NETOPT_ENABLE;
    dev->driver->set(dev, NETOPT_RX_START_IRQ, &tell_start, sizeof(tell_start));
    dev->driver->set(dev, NETOPT_TX_START_IRQ, &tell_start, sizeof(tell_start));
    dev->driver->set(dev, NETOPT_TX_END_IRQ, &tell_start, sizeof(tell_start));

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

        LOG_DEBUG("Waiting for incoming messages\n");
        msg_receive(&msg);

        LOG_DEBUG("Got msg of type %p\n", (void*)((uint32_t)msg.type));

        /* Handle NETDEV, NETAPI, RTT and TIMEOUT messages */
        switch (msg.type) {

        /* RTT raised an interrupt */
        case LWMAC_EVENT_RTT_TYPE:
            rtt_handler(msg.content.value);
            lwmac_schedule_update();
            break;

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
