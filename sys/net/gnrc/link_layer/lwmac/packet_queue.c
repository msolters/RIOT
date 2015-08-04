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
 * @brief       Wrapper for priority_queue that holds gnrc_pktsnip_t* and is
 *              aware of it's length.
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include "net/gnrc.h"
#include "net/gnrc/lwmac/packet_queue.h"
#include "panic.h"

/******************************************************************************/

gnrc_pktsnip_t* packet_queue_pop(packet_queue_t* q)
{
    if(!q || (q->length == 0))
        return NULL;
    priority_queue_node_t* head = priority_queue_remove_head(&(q->queue));
    gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) head->data;
    free(head);
    q->length--;
    return pkt;
}

/******************************************************************************/

gnrc_pktsnip_t* packet_queue_head(packet_queue_t* q)
{
    if(!q || (q->length == 0))
        return NULL;
    return (gnrc_pktsnip_t*) q->queue.first->data;
}
/******************************************************************************/

priority_queue_node_t*
packet_queue_push(packet_queue_t* q, gnrc_pktsnip_t* snip, uint32_t priority)
{
    assert(sizeof(unsigned int) == sizeof(gnrc_pktsnip_t*));

    if(!q || !snip)
        return NULL;

    priority_queue_node_t* node = malloc(sizeof(priority_queue_node_t));

    if(node)
    {
        *node = (priority_queue_node_t) PRIORITY_QUEUE_NODE_INIT;
        node->data = (unsigned int) snip;
        node->priority = priority;

        priority_queue_add(&(q->queue), node);
        q->length++;
    }
    return node;
}

/******************************************************************************/

void packet_queue_flush(packet_queue_t* q)
{
    if(q->length == 0)
        return;

    priority_queue_node_t* node;
    while( (node = priority_queue_remove_head(&(q->queue))) )
    {
        gnrc_pktbuf_release((gnrc_pktsnip_t*) node->data);
        free(node);
    }
    q->length = 0;
}
