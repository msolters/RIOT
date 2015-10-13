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

#include <xtimer.h>
#include <net/gnrc/lwmac/lwmac.h>

#include "include/timeout.h"
#include "include/lwmac_types.h"

/******************************************************************************/

char* lwmac_timeout_names[] = {
    [TIMEOUT_DISABLED]              = "TIMEOUT_DISABLED",
    [TIMEOUT_WR]                    = "TIMEOUT_WR",
    [TIMEOUT_NO_RESPONSE]           = "TIMEOUT_NO_RESPONSE",
    [TIMEOUT_WA]                    = "TIMEOUT_WA",
    [TIMEOUT_DATA]                  = "TIMEOUT_DATA",
    [TIMEOUT_WAIT_FOR_DEST_WAKEUP]  = "TIMEOUT_WAIT_FOR_DEST_WAKEUP"
};

/******************************************************************************/

static inline void _lwmac_clear_timeout(lwmac_timeout_t* timeout)
{
    xtimer_remove(&(timeout->timer));
    timeout->type = TIMEOUT_DISABLED;
}

/******************************************************************************/

/* Return index >= 0 if found, -ENONENT if not found */
static int _lwmac_find_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type)
{
    int i;
    for(i = 0; i < LWMAC_TIMEOUT_COUNT; i++)
    {
        if(lwmac->timeouts[i].type == type)
            return i;
    }
    return -ENOENT;
}

/******************************************************************************/

inline bool lwmac_timeout_is_running(lwmac_t* lwmac, lwmac_timeout_type_t type)
{
    return (_lwmac_find_timeout(lwmac, type) >= 0);
}

/******************************************************************************/

bool lwmac_timeout_is_expired(lwmac_t* lwmac, lwmac_timeout_type_t type)
{
    int index = _lwmac_find_timeout(lwmac, type);
    if(index >= 0) {
        if(lwmac->timeouts[index].expired)
            _lwmac_clear_timeout(&lwmac->timeouts[index]);
        return lwmac->timeouts[index].expired;
    }
    return false;
}

/******************************************************************************/

lwmac_timeout_t* _lwmac_acquire_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type)
{
    if(lwmac_timeout_is_running(lwmac, type))
        return NULL;

    int i;
    for(i = 0; i < LWMAC_TIMEOUT_COUNT; i++)
    {
        if(lwmac->timeouts[i].type == TIMEOUT_DISABLED)
        {
            lwmac->timeouts[i].type = type;
            return &lwmac->timeouts[i];
        }
    }
    return NULL;
}

/******************************************************************************/

void lwmac_clear_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type)
{
    int index = _lwmac_find_timeout(lwmac, type);
    if(index >= 0)
        _lwmac_clear_timeout(&lwmac->timeouts[index]);
}

/******************************************************************************/

void lwmac_set_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type, timex_t* interval)
{
    lwmac_timeout_t* timeout;
    if( (timeout = _lwmac_acquire_timeout(lwmac, type)) )
    {
        timeout->expired = false;
        timeout->msg.type = LWMAC_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void*) timeout;
        xtimer_set_msg(&(timeout->timer), interval->microseconds,
                       &(timeout->msg), lwmac->pid);
    }
}

/******************************************************************************/

void lwmac_reset_timeouts(lwmac_t* lwmac)
{
    int i;
    for(i = 0; i < LWMAC_TIMEOUT_COUNT; i++)
    {
        if(lwmac->timeouts[i].type != TIMEOUT_DISABLED)
            _lwmac_clear_timeout(&lwmac->timeouts[i]);
    }
}
