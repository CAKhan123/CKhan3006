// Christopher Khan - 816031052

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "hw_timer.h"

#include <stdarg.h>
#include <stdint.h>

static void serial_init(unsigned long baud) {
  uart_init((UartBautRate)baud, (UartBautRate)baud);
}
static void serial_printf(const char *fmt, ...) {
  char b[256];
  va_list ap; va_start(ap, fmt);
  ets_vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  os_printf("%s", b);
}

/* timebase + sleep */
typedef struct { volatile uint32_t ms, sec; } timebase_t;
static timebase_t tb;

static void IRAM_ATTR tick_isr(void) {
  uint32_t m = tb.ms + 1u;
  tb.ms = m;
  if ((m % 1000u) == 0u) tb.sec++;
}
static void timebase_init(void) {
  ETS_INTR_LOCK(); tb.ms = 0; tb.sec = 0; ETS_INTR_UNLOCK();
  hw_timer_init(FRC1_SOURCE, 1);        // auto-reload
  hw_timer_set_func(tick_isr);
  hw_timer_arm(1000);                    // 1 ms
}
static inline uint32_t time_ms(void) { return tb.ms; }
static inline int deadline_reached(uint32_t t) { return (int32_t)(time_ms() - t) >= 0; }
static inline void nap(void){ system_soft_wdt_feed(); os_delay_us(200); }
static void sleep_until(uint32_t t) { while (!deadline_reached(t)) nap(); }
static void sleep_ms(uint32_t d) { uint32_t t0 = time_ms(); sleep_until(t0 + d); (void)d; }

/* demo tasks */
static void task_A(void){ serial_printf("[A] %lu ms\r\n", (unsigned long)time_ms()); }
static void task_B(void){ serial_printf("[B] %lu ms\r\n", (unsigned long)time_ms()); }
static void task_C(void){ serial_printf("[C] %lu ms\r\n", (unsigned long)time_ms()); }
static void task_D(void){ serial_printf("[D] %lu ms\r\n", (unsigned long)time_ms()); }

/* cyclic executive */
#define FRAME_MS 50u
typedef void (*task_fn)(void);
typedef struct { const task_fn *v; uint8_t n; } frame_t;

static const task_fn F0[] = { task_A, task_B };
static const task_fn F1[] = { task_A };
static const task_fn F2[] = { task_A, task_C };
static const task_fn F3[] = { task_A };
static const task_fn F4[] = { task_A, task_B };
static const task_fn F5[] = { task_A };
static const task_fn F6[] = { task_A, task_C };
static const task_fn F7[] = { task_A };
static const task_fn F8[] = { task_A, task_B };
static const task_fn F9[] = { task_A, task_D };

static const frame_t S[] = {
  { F0, sizeof F0 / sizeof *F0 }, { F1, sizeof F1 / sizeof *F1 },
  { F2, sizeof F2 / sizeof *F2 }, { F3, sizeof F3 / sizeof *F3 },
  { F4, sizeof F4 / sizeof *F4 }, { F5, sizeof F5 / sizeof *F5 },
  { F6, sizeof F6 / sizeof *F6 }, { F7, sizeof F7 / sizeof *F7 },
  { F8, sizeof F8 / sizeof *F8 }, { F9, sizeof F9 / sizeof *F9 },
};

static void run_frame(const frame_t *f){ for (uint8_t i=0;i<f->n;i++) f->v[i](); }

static void ICACHE_FLASH_ATTR app_start(void) {
  serial_init(115200);
  timebase_init();
  // wifi_set_opmode(NULL_MODE); // uncomment if you don't need Wi-Fi

  serial_printf("\r\nESP8266 cyclic exec, frame=%u ms\r\n", FRAME_MS);

  uint32_t t = ((time_ms()/FRAME_MS)+1u)*FRAME_MS;
  uint8_t ix = 0, N = (uint8_t)(sizeof S / sizeof *S);

  for (;;) {
    sleep_until(t);
    run_frame(&S[ix]);
    t += FRAME_MS;
    ix = (uint8_t)((ix + 1u) % N);
  }
}

void ICACHE_FLASH_ATTR user_init(void) {
  system_init_done_cb(app_start);
}
