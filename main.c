#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define IN1          GPIO_NUM_19
#define IN2          GPIO_NUM_18
#define IN3          GPIO_NUM_5
#define IN4          GPIO_NUM_17
#define PIN_LEDS     GPIO_NUM_23   
#define PIN_LAMP     GPIO_NUM_32

#define LDR_MIN_MV   519.0f
#define LDR_MAX_MV   3200.0f
#define TEMP_OFFSET  0.0f
#define LAMP_INVERT  0
#define LEDS_INVERT  1   // El opto invierte la lógica del PWM

// Motor
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_HEAT,
    MOTOR_VENT_LOW,
    MOTOR_VENT_MED,
    MOTOR_VENT_HIGH,
} motor_state_t;

typedef struct { int in1, in2, in3, in4; } step_t;

static const step_t seq[8] = {
    {1,0,0,0}, {1,0,1,0}, {0,0,1,0}, {0,1,1,0},
    {0,1,0,0}, {0,1,0,1}, {0,0,0,1}, {1,0,0,1}
};

// Estado inicial
static volatile motor_state_t g_motor_state = MOTOR_STOP;
static volatile float g_tc   = 25.0f;
static volatile float g_temp = 0.0f;
static volatile float g_luz  = 0.0f;
static volatile int   g_leds = 0;
static volatile int   g_lamp = 0;

// ADC
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_lm35;
static adc_cali_handle_t cali_ldr;

static void adc_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t cfg_lm35 = {
        .atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &cfg_lm35);

    adc_oneshot_chan_cfg_t cfg_ldr = {
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &cfg_ldr);

    adc_cali_line_fitting_config_t cal1 = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_line_fitting(&cal1, &cali_lm35);

    adc_cali_line_fitting_config_t cal2 = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_line_fitting(&cal2, &cali_ldr);
}

static float lm35_read(void) {
    int raw, mv, suma = 0;
    for (int i = 0; i < 64; i++) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &raw);
        adc_cali_raw_to_voltage(cali_lm35, raw, &mv);
        suma += mv;
    }
    return ((suma / 64.0f) / 10.0f) - TEMP_OFFSET;
}

static float ldr_read_percent(void) {
    int raw, mv, suma = 0;
    for (int i = 0; i < 64; i++) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &raw);
        adc_cali_raw_to_voltage(cali_ldr, raw, &mv);
        suma += mv;
    }
    float p = ((suma / 64.0f) - LDR_MIN_MV) / (LDR_MAX_MV - LDR_MIN_MV) * 100.0f;
    if (p < 0)   p = 0;
    if (p > 100) p = 100;
    return p;
}

// Motor
static void motor_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL<<IN1)|(1ULL<<IN2)|(1ULL<<IN3)|(1ULL<<IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level(IN1, 0); gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0); gpio_set_level(IN4, 0);
}

static void motor_off(void) {
    gpio_set_level(IN1, 0); gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0); gpio_set_level(IN4, 0);
}

static void motor_task(void *arg) {
    int idx = 0;
    int counter = 0;
    while (1) {
        motor_state_t s = g_motor_state;

        if (s == MOTOR_STOP) {
            motor_off();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int clockwise;
        uint32_t delay_us;
        switch (s) {
            case MOTOR_HEAT:      clockwise = 1; delay_us = 10000; break;
            case MOTOR_VENT_LOW:  clockwise = 0; delay_us = 10000; break;
            case MOTOR_VENT_MED:  clockwise = 0; delay_us = 3333;  break;
            case MOTOR_VENT_HIGH: clockwise = 0; delay_us = 1667;  break;
            default: motor_off(); vTaskDelay(pdMS_TO_TICKS(50)); continue;
        }

        gpio_set_level(IN1, seq[idx].in1);
        gpio_set_level(IN2, seq[idx].in2);
        gpio_set_level(IN3, seq[idx].in3);
        gpio_set_level(IN4, seq[idx].in4);
        idx = clockwise ? (idx + 1) % 8 : (idx + 7) % 8;
        esp_rom_delay_us(delay_us);

        if (++counter >= 100) {
            counter = 0;
            vTaskDelay(1);
        }
    }
}

// Lámpara
static void lamp_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LAMP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_LAMP, LAMP_INVERT ? 1 : 0);
}

static void lamp_set(int on) {
    g_lamp = on ? 1 : 0;
    int level = LAMP_INVERT ? !g_lamp : g_lamp;
    gpio_set_level(PIN_LAMP, level);
}

// LEDS PWM
#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_DUTY_RES  LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ   500     // bajo para que el opto responda

static void leds_init(void) {
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .gpio_num   = PIN_LEDS,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ccfg);
}

static void leds_set_percent(int percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    g_leds = percent;

    int real = LEDS_INVERT ? (100 - percent) : percent;
    uint32_t duty = (real * 1023) / 100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Logica de control
static void aplicar_temperatura(float T, float Tc) {
    if (T < Tc - 1.0f) {
        g_motor_state = MOTOR_HEAT;
        lamp_set(1);
    } else if (T <= Tc + 1.0f) {
        g_motor_state = MOTOR_STOP;
        lamp_set(0);
    } else if (T < Tc + 3.0f) {
        g_motor_state = MOTOR_VENT_LOW;
        lamp_set(0);
    } else if (T <= Tc + 5.0f) {
        g_motor_state = MOTOR_VENT_MED;
        lamp_set(0);
    } else {
        g_motor_state = MOTOR_VENT_HIGH;
        lamp_set(0);
    }
}

static void aplicar_iluminacion(float ni) {
    int p;
    if      (ni < 20) p = 100;
    else if (ni < 30) p = 80;
    else if (ni < 40) p = 60;
    else if (ni < 60) p = 50;
    else if (ni < 80) p = 30;
    else              p = 0;
    leds_set_percent(p);
}

// Serial
#define UART_PORT     UART_NUM_0
#define UART_BUF_SIZE 256

static void serial_init(void) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
}

static void serial_task(void *arg) {
    char buf[64];
    int idx = 0;
    uint8_t ch;

    while (1) {
        int len = uart_read_bytes(UART_PORT, &ch, 1, pdMS_TO_TICKS(50));
        if (len <= 0) continue;

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                if (strncmp(buf, "SET_TEMP:", 9) == 0) {
                    int v = atoi(buf + 9);
                    if (v > 0 && v < 100) {
                        g_tc = (float)v;
                        printf("\n>> Tc actualizado a %.1f C\n\n", g_tc);
                    } else {
                        printf("\n>> Valor invalido\n\n");
                    }
                }
                idx = 0;
            }
        } else if (idx < 63) {
            buf[idx++] = (char)ch;
        }
    }
}

void app_main(void) {
    adc_init();
    motor_init();
    lamp_init();
    leds_init();
    serial_init();

    xTaskCreate(motor_task,  "motor",  2048, NULL, 5, NULL);
    xTaskCreate(serial_task, "serial", 2048, NULL, 3, NULL);

    printf("\n========================================\n");
    printf("  SISTEMA DOMOTICO - LAB 3\n");
    printf("========================================\n");
    printf("Comando: SET_TEMP:XX  (ej: SET_TEMP:25)\n");
    printf("----------------------------------------\n\n");

    const char *estado_motor[] = {
        "STOP        ",
        "HEAT (CW)   ",
        "VENT-LOW    ",
        "VENT-MED    ",
        "VENT-HIGH   "
    };

    while (1) {
        g_temp = lm35_read();
        g_luz  = ldr_read_percent();

        aplicar_temperatura(g_temp, g_tc);
        aplicar_iluminacion(g_luz);

        printf("Tc=%4.1fC | T=%4.1fC | Luz=%5.1f%% | LEDs=%3d%% | Lampara=%s | Motor=%s\n",
               g_tc, g_temp, g_luz, g_leds,
               g_lamp ? "ON " : "OFF",
               estado_motor[g_motor_state]);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

