/* =============================================================
 * ringbuf.c — UART RX 环形缓冲区
 *   生产者: UART RX 中断（每收 1 byte 调 push）
 *   消费者: 主循环（pop_frame / has_frame）
 * ============================================================= */
#include <string.h>
#include "ringbuf.h"
#include "chassis_config.h"

void ringbuf_init(ringbuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->len  = 0;
    memset(rb->data, 0, RINGBUF_SIZE);
}

int ringbuf_push(ringbuf_t *rb, uint8_t byte)
{
    if (rb->len >= RINGBUF_SIZE) return -1;   /* 满了 */
    rb->data[rb->tail] = byte;
    rb->tail = (rb->tail + 1) % RINGBUF_SIZE;
    rb->len++;
    return 0;
}

/* 把当前缓存内容拷出去，同时消费掉
 * 返回实际字节数
 */
int ringbuf_flush_to(ringbuf_t *rb, uint8_t *dest, int max_len)
{
    int n = (rb->len < max_len) ? rb->len : max_len;
    for (int i = 0; i < n; i++) {
        dest[i] = rb->data[rb->head];
        rb->head = (rb->head + 1) % RINGBUF_SIZE;
    }
    rb->len -= n;
    return n;
}

/* 查找 \r\n 帧结束符。有就返回帧长度（不含 \r\n），没就返回 0 */
int ringbuf_has_ascii_frame(ringbuf_t *rb)
{
    int idx = rb->head;
    for (int i = 0; i < rb->len; i++) {
        if (rb->data[idx] == '\r') {
            /* 检查下一位是不是 \n */
            int next = (idx + 1) % RINGBUF_SIZE;
            if ((i + 1 < rb->len) && rb->data[next] == '\n') {
                return i;   /* 帧内容长度（不含 \r\n）*/
            }
        }
        idx = (idx + 1) % RINGBUF_SIZE;
    }
    return 0;
}

/* 二进制帧查找 SOF + EOF */
#define BIN_SOF 0xA5
#define BIN_EOF 0x5A
int ringbuf_has_bin_frame(ringbuf_t *rb)
{
    int idx = rb->head;
    for (int i = 0; i < rb->len; i++) {
        if (rb->data[idx] == BIN_SOF) {
            /* 二进制帧固定 13 字节：SOF(1)+CMD(1)+v(4)+w(4)+CRC(2)+EOF(1) */
            if (rb->len - i >= 13) {
                int eof_idx = (idx + 12) % RINGBUF_SIZE;
                if (rb->data[eof_idx] == BIN_EOF) return 13;
            }
        }
        idx = (idx + 1) % RINGBUF_SIZE;
    }
    return 0;
}
