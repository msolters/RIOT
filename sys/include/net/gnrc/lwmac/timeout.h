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
 * @brief       Timeout handling.
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#ifndef LWMAC_TIMEOUT_H
#define LWMAC_TIMEOUT_H

#include "timex.h"
#include "lwmac.h"
#include "priority_queue.h"
#include "net/gnrc/pkt.h"

#ifdef __cplusplus
extern "C" {
#endif


void lwmac_set_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type, timex_t* interval);

void lwmac_clear_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type);

bool lwmac_timeout_is_running(lwmac_t* lwmac, lwmac_timeout_type_t type);

bool lwmac_timeout_is_expired(lwmac_t* lwmac, lwmac_timeout_type_t type);

void lwmac_reset_timeouts(lwmac_t* lwmac);

#ifdef __cplusplus
}
#endif

#endif /* LWMAC_TIMEOUT_H */
