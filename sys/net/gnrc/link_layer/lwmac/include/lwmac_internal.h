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
 * @brief       Internal functions if LWMAC
 * @{
 *
 * @file
 * @brief       Interface definition for internal functions of LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_LWMAC_INTERNAL_H_
#define GNRC_LWMAC_INTERNAL_H_

#include <stdint.h>
#include "periph/rtt.h"
#include "lwmac_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* @brief   Next RTT event must be at least this far in the future
 *
 * When setting an RTT alarm to short in the future it could be possible that
 * the counter already passed the calculated alarm before it could be set. This
 * margin will be applied when using `_next_inphase_event()`.
 */
#define LWMAC_RTT_EVENT_MARGIN_TICKS    ( RTT_MS_TO_TICKS(2) )

/* @brief Extract the destination address out of an GNRC_NETTYPE_NETIF pktsnip
 *
 * @param[in]   pkt                 pktsnip from whom to extract
 * @param[out]  pointer_to_addr     pointer to address will be stored here
 *
 * @return                          length of destination address
 */
int _get_dest_address(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[]);

/* @brief Find the first pktsnip of @p type
 *
 * Will search linearly through the packet buffer @p pkt and yield
 * gnrc_pktsnip_t::data of the first pktsnip match the type @p type.
 *
 * @param[in]   pkt     pktsnip that will be searched
 * @param[in]   type    type to search for
 *
 * @return              pointer to data, NULL is not found
 */
void* _gnrc_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type);

/* @brief Check packet if address matches and is of expected type
 *
 * Check if @p pkt type is @p expected_type and then check if destination
 * address matches own address in @p lwmac.
 *
 * TODO: also accept broadcast
 *
 * @param[in]   pkt             packet that will be checked
 * @param[in]   expected_type   required type
 * @param[in]   lwmac           lwmac state that stores own address
 *
 * @return                      whether the packet should be accepted or not
 */
bool _accept_packet(gnrc_pktsnip_t* pkt, lwmac_frame_type_t expected_type, lwmac_t* lwmac);

/* @brief Shortcut to get the state of netdev
 *
 * @param[in]   lwmac           lwmac state that stores netdev pointer
 *
 * @return                      state of netdev
 */
netopt_state_t _get_netdev_state(lwmac_t* lwmac);

/* @brief Shortcut to set the state of netdev
 *
 * @param[in]   lwmac           lwmac state that stores netdev pointer
 * @param[in]   devstate        new state for netdev
 */
void _set_netdev_state(lwmac_t* lwmac, netopt_state_t devstate);

/* TX queue handling */
int _find_neighbour(lwmac_t* lwmac, uint8_t* dst_addr, int addr_len);
int _free_neighbour(lwmac_t* lwmac);
int _alloc_neighbour(lwmac_t* lwmac);
void _init_neighbour(lwmac_tx_neighbour_t* neighbour, uint8_t* addr, int len);

static inline lwmac_tx_neighbour_t* _get_neighbour(lwmac_t* lwmac, unsigned int id)
{
    return &(lwmac->tx.neighbours[id]);
}

/* RTT phase calculation */
uint32_t _ticks_to_phase(uint32_t ticks);
uint32_t _phase_to_ticks(uint32_t phase);
uint32_t _phase_now(void);
uint32_t _ticks_until_phase(uint32_t phase);

int _next_tx_neighbour(lwmac_tx_neighbour_t neighbours[]);
int _time_until_tx_us(lwmac_t* lwmac);
bool _queue_tx_packet(lwmac_t* lwmac,  gnrc_pktsnip_t* pkt);
uint32_t _next_inphase_event(uint32_t last, uint32_t interval);


#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_INTERNAL_H_ */
/** @} */
