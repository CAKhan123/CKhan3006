#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"     
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"



#define LED_PIN                		2		// pin for LED
#define UART_BAUD              		115200		// adjust

// Task priorities (distinct)
#define PRIO_GPIO_OWNER        4            
#define PRIO_LED_ON            3
#define PRIO_LED_OFF           2
#define PRIO_STATUS            1



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



typedef enum { LED_CMD_HIGH, LED_CMD_LOW } led_cmd_t;
static QueueHandle_t g_ledQ;                 // created in app_main()



// Task A:
static void taskLedOn(void *arg) {
    (void)arg;
    for (;;) {
        led_cmd_t c = LED_CMD_HIGH;
        // send request to the GPIO owner, blocks if queue full
        xQueueSend(g_ledQ, &c, portMAX_DELAY);

        // active wait 0.5 s
        const uint32_t start_us = system_get_time();
        while ((uint32_t)(system_get_time() - start_us) < 500000U) {
            // spin
        }

        taskYIELD();
    }
}

// Task B: 
static void taskLedOff(void *arg) {
    (void)arg;
    for (;;) {
        led_cmd_t c = LED_CMD_LOW;
        xQueueSend(g_ledQ, &c, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task C: 
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

// New Task D:
static void taskGpio(void *arg) {
    (void)arg;
    led_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(g_ledQ, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd == LED_CMD_HIGH) led_write(1);
            else                     led_write(0);
        }
    }
}



void app_main(void) {
    // Init UART0 to 115200 so printf() goes to USB serial
    uart_div_modify(0, UART_CLK_FREQ / UART_BAUD);

    led_gpio_init();
    led_write(0);

    // Create mutex for the shared LED pin
    g_ledMutex = xSemaphoreCreateMutex();

    // Create tasks with distinct priorities
    BaseType_t ok;
    ok = xTaskCreate(taskLedOn,  "LED_ON",   1024, NULL, PRIO_LED_ON,  NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(taskLedOff, "LED_OFF",  1024, NULL, PRIO_LED_OFF, NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(taskStatus, "STATUS",   2048, NULL, PRIO_STATUS,  NULL);
    configASSERT(ok == pdPASS);
}
