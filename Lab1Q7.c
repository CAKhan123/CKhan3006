// Christopher Khan - 816031052

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "hw_timer.h"

#include <stdint.h>
#include <stdarg.h>

/* UART */
static void serial_init(unsigned long baud){
  uart_init((UartBautRate)baud, (UartBautRate)baud);
}
static void print(const char *fmt, ...){
  char buf[160];
  va_list ap; va_start(ap, fmt);
  ets_vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  os_printf("%s", buf);
}

/* 1 kHz timebase */
static volatile uint32_t g_ms = 0, g_sec = 0;
static void IRAM_ATTR tick_isr(void){
  uint32_t m = g_ms + 1u;
  g_ms = m;
  if ((m % 1000u) == 0u) g_sec++;
}
static void timebase_init(void){
  g_ms = 0; g_sec = 0;
  hw_timer_init(FRC1_SOURCE, 1);
  hw_timer_set_func(tick_isr);
  hw_timer_arm(1000); /* 1 ms */
}
static inline uint32_t ms_now(void){ return g_ms; }

/* LED (GPIO2, active-low by default) */
#define LED_GPIO        2
#define LED_ACTIVE_LOW  1
static void led_init(void){
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
  gpio_output_set(0, 0, (1<<LED_GPIO), 0);
}
static void led_set(int on){
#if LED_ACTIVE_LOW
  if (on) gpio_output_set(0, (1<<LED_GPIO), 0, 0);
  else    gpio_output_set((1<<LED_GPIO), 0, 0, 0);
#else
  if (on) gpio_output_set((1<<LED_GPIO), 0, 0, 0);
  else    gpio_output_set(0, (1<<LED_GPIO), 0, 0);
#endif
}

/* RX */
static int rx_byte(uint8_t *c){ return uart_rx_one_char(c) == OK; }

/* FSM */
typedef enum { ST_OFF = 0, ST_ON = 1 } state_t;
#define VALID_CHAR   'a'
#define TICK_MS      10u
#define DEBOUNCE_MS  500u

static state_t st = ST_OFF;
static uint32_t last_ms = 0;

static void fsm_step(void){
  uint8_t c;
  uint32_t now = ms_now();

  switch (st){
    case ST_OFF:
      if (rx_byte(&c) && c == (uint8_t)VALID_CHAR){
        if ((now - last_ms) >= DEBOUNCE_MS){ st = ST_ON; last_ms = now; }
      }
      led_set(0);
      break;

    case ST_ON:
      if (rx_byte(&c) && c == (uint8_t)VALID_CHAR){
        if ((now - last_ms) >= DEBOUNCE_MS){ st = ST_OFF; last_ms = now; }
      }
      led_set(1);
      break;
  }
}

/* Main */
static void ICACHE_FLASH_ATTR app_start(void){
  serial_init(115200);
  timebase_init();
  led_init();
  print("\r\nDebounce FSM ready. Valid char: '%c'\r\n", VALID_CHAR);

  uint32_t next = ((ms_now()/TICK_MS)+1u)*TICK_MS;
  for (;;){
    if ((int32_t)(ms_now() - next) >= 0){
      fsm_step();
      next += TICK_MS;
    }
    system_soft_wdt_feed();
    os_delay_us(200);
  }
}

void ICACHE_FLASH_ATTR user_init(void){
  system_init_done_cb(app_start);
}
