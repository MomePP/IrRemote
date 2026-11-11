#pragma once

// Orders a write against a reader on the other core or in an interrupt: memw on Xtensa (ESP32, S2, S3), a full fence on
// the RISC-V chips, which have no memw.
inline void irMemoryBarrier(void)
{
#ifdef __XTENSA__
    __asm__ volatile("memw" ::: "memory");
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}
