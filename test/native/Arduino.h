// Minimal Arduino.h shim so storage.cpp can be compiled and exercised on the
// host. Only the pieces storage.cpp actually touches.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// storage.cpp invalidates the D-cache before reading memory-mapped flash.
// The host flash model is plain RAM, so this is a no-op.
static inline void SCB_InvalidateDCache_by_Addr(void *addr, int32_t len)
{
    (void)addr;
    (void)len;
}

#define __DCACHE_PRESENT 1
