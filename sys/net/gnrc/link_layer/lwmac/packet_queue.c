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

#include <net/gnrc.h>
#include <net/gnrc/lwmac/lwmac.h>
#include <net/gnrc/lwmac/packet_queue.h>

static priority_queue_node_t _packet_queue_nodes[LWMAC_TX_QUEUE_SIZE];

/******************************************************************************/

static priority_queue_node_t* _alloc_node(gnrc_pktsnip_t *pkt)
{
    assert(sizeof(unsigned int) == sizeof(gnrc_pktsnip_t*));

    for (size_t i = 0; i < sizeof(_packet_queue_nodes) / sizeof(priority_queue_node_t); i++) {
        if( (_packet_queue_nodes[i].data == 0) &&
            (_packet_queue_nodes[i].next == NULL))
        {
            _packet_queue_nodes[i].data = (unsigned int) pkt;
            return &(_packet_queue_nodes[i]);
        }
    }

    return NULL;
}

/******************************************************************************/

static inline void _free_node(priority_queue_node_t *node)
{
    if(node) {
        node->data = 0;
        node->next = NULL;
    }
}

/******************************************************************************/

gnrc_pktsnip_t* packet_queue_pop(packet_queue_t* q)
{
    if(!q || (q->length == 0))
        return NULL;
    priority_queue_node_t* head = priority_queue_remove_head(&(q->queue));
    gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) head->data;
    _free_node(head);
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
    if(!q || !snip)
        return NULL;

    priority_queue_node_t* node = _alloc_node(snip);

    if(node)
    {
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
        _free_node(node);
    }
    q->length = 0;
}
