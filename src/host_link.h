#pragma once
#include <stdbool.h>
#include <stdint.h>

// Which protocol the host speaks, and whether its link is up (main.cpp; hardware-free, host-tested
// in test/test_host_link).
//
// The main loop runs both protocols on every pass, and each one reports whether it saw a heartbeat
// on that pass and whether its link is down (bus_link.h latches the loss). A heartbeat sets the host
// type (bus_host_device_type) to its protocol, until a heartbeat of the other one. Only the protocol
// the host speaks decides: the other one never gets a heartbeat and must not mask a lost link.
// Before the first heartbeat the host type is none and the BMCU stays offline. Offline, main() runs
// Motion_control_run(-1) (motors stopped) and pending NVM writes wait for the link to come back.

// bus_host_device_type values (also the AMS device address on BambuBus).
#define host_device_type_none 0x0000
#define host_device_type_ahub 0x0001
#define host_device_type_ams 0x0700

// One protocol's report for this pass.
typedef struct
{
    bool heartbeat; // a heartbeat was reported
    bool error;     // the link is down: not seen yet (BambuBus), or lost
} host_link_report_t;

// Updates the host type from this pass's reports; returns whether the BMCU is offline.
static inline bool host_link_offline(uint16_t *host_type, host_link_report_t bambubus, host_link_report_t ahub)
{
    if (bambubus.heartbeat)
        *host_type = host_device_type_ams;

    if (ahub.heartbeat)
        *host_type = host_device_type_ahub;

    bool offline = true;
    if (*host_type == host_device_type_ams)
        offline = bambubus.error;
    else if (*host_type == host_device_type_ahub)
        offline = ahub.error;
    return offline;
}
