#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include "chassis_config.h"

typedef struct {
    uint8_t  data[RINGBUF_SIZE];
    int      head;
    int      tail;
    int      len;
} ringbuf_t;

void ringbuf_init(ringbuf_t *rb);
int  ringbuf_push(ringbuf_t *rb, uint8_t byte);
int  ringbuf_flush_to(ringbuf_t *rb, uint8_t *dest, int max_len);
int  ringbuf_has_ascii_frame(ringbuf_t *rb);
int  ringbuf_has_bin_frame(ringbuf_t *rb);

#endif
