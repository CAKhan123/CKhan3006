#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"



#define LED_PIN                		2		// pin for LED
#define UART_BAUD              		115200		// adjust
#define PRIO_LED_ON           		3
#define PRIO_LED_OFF           		2
#define PRIO_STATUS            		1
#define configUSE_TRACE_FACILITY        1
#define traceTASK_SWITCHED_IN()  do { printf("[TRACE] Switched IN: %s\r\n", pcTaskGetName(NULL)); } while(0)
#define traceTASK_SWITCHED_OUT() do { printf("[TRACE] Switched OUT: %s\r\n", pcTaskGetName(NULL)); } while(0)
#define traceBLOCKING_ON_MUTEX(pxMutex) do { printf("[TRACE] %s blocked on mutex\r\n", pcTaskGetName(NULL)); } while(0)
#define traceGIVE_MUTEX(pxMutex) do { printf("[TRACE] %s released mutex\r\n", pcTaskGetName(NULL)); } while(0)



static SemaphoreHandle_t g_ledMutex;

static void led_gpio_init(void) {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
    gpio_output_set(0, 0, (1 << LED_PIN), 0);    // enable output on GPIO2
}

static inline void led_write(int level) {
    if (level) {
        GPIO_OUTPUT_SET(LED_PIN, 1);
    } else {
        GPIO_OUTPUT_SET(LED_PIN, 0);
    }
}



// Task A
static void taskLedOn(void *arg) {
    (void)arg;
    for (;;) {
        xSemaphoreTake(g_ledMutex, portMAX_DELAY);
        led_write(1);
        const uint32_t start_us = system_get_time();
        while ((uint32_t)(system_get_time() - start_us) < 500000U) {
        }
        xSemaphoreGive(g_ledMutex);
        taskYIELD();
    }
}

// Task B
static void taskLedOff(void *arg) {
    (void)arg;
    for (;;) {
        xSemaphoreTake(g_ledMutex, portMAX_DELAY);
        led_write(0);
        xSemaphoreGive(g_ledMutex);
        vTaskDelay(pdMS_TO_TICKS(1000));   // 1 s
    }
}

// Task C
static void taskStatus(void *arg) {
    (void)arg;
    uint32_t tick = 0;
    for (;;) {
        printf("[Status] t=%lu s, free heap ~ %u bytes\r\n",
               (unsigned long)tick++,
               (unsigned int)system_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



void app_main(void) {
    // Init UART0 to 115200 so printf() goes to USB serial
    uart_div_modify(0, UART_CLK_FREQ / UART_BAUD);

    led_gpio_init();
    led_write(0);

    // Create mutex for the shared LED pin
    g_ledMutex = xSemaphoreCreateMutex();
    configASSERT(g_ledMutex != NULL);

    // Create tasks with distinct priorities
    BaseType_t ok;
    ok = xTaskCreate(taskLedOn,  "LED_ON",   1024, NULL, PRIO_LED_ON,  NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(taskLedOff, "LED_OFF",  1024, NULL, PRIO_LED_OFF, NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(taskStatus, "STATUS",   2048, NULL, PRIO_STATUS,  NULL);
    configASSERT(ok == pdPASS);
}
