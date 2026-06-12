#pragma once

#define PLATFORM_OW485

#ifdef PLATFORM_OW485
	#include <Arduino.h>
	#include <platform-ow485.h>
#else
	#warning "Undefined or invalid platform"
#endif
