#pragma once
// bus_host_device_type values (_bus_hardware.h): the protocol the host speaks, also the AMS device
// address on BambuBus. main() sets the type from the protocol that answers and decides the offline
// state inline (src/main.cpp, modelled in test/test_host_link).

#define host_device_type_none 0x0000
#define host_device_type_ahub 0x0001
#define host_device_type_ams 0x0700
