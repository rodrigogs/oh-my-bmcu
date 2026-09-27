#pragma once
// Host stand-in for the CH32V20x SDK header, which src/ws2812.h includes (through app_api.h) only
// for GPIO_TypeDef. The test never touches a GPIO, so an incomplete type is enough.

typedef struct host_gpio GPIO_TypeDef;
