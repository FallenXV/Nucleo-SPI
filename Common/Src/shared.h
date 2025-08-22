#ifndef COMMON_SHARED_H
#define COMMON_SHARED_H

#include <stdint.h>

#define SHM_BASE   (0x30040000UL)     // your CubeMX “ADC Output” base
#define RB_CAP     256

typedef struct {
  volatile uint32_t head;
  volatile uint32_t tail;
  struct {
    uint32_t status;
    int32_t  ch[4];                   // 24-bit ADC values sign-extended to 32
  } sample[RB_CAP];
} __attribute__((aligned(32))) adc_ring_t;

// one canonical pointer both cores use
static inline volatile adc_ring_t * SHM(void) {
  return (volatile adc_ring_t *)SHM_BASE;
}

#endif /* COMMON_SHARED_H */