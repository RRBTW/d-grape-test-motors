/* ============================================================
 *  main.c — d-grape-test-motors (BTS7960B, без энкодеров)
 *
 *  Управление через USB CDC:
 *    w / s      — +0.1 / -0.1 скважность оба мотора
 *    a / d      — поворот влево / вправо
 *    пробел     — стоп
 *    left V right V   — задать скважность [-1.0 .. 1.0] (Enter)
 *    stop             — стоп (Enter)
 *    pid kp V         — изменить Kp (Enter)
 *    pid ki V         — изменить Ki (Enter)
 *    pid kd V         — изменить Kd (Enter)
 *
 *  Вывод 5 Гц: L_tgt  L_duty  R_tgt  R_duty
 * ============================================================ */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "motors.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── HAL handle ─────────────────────────────────────────────*/
TIM_HandleTypeDef htim2;

/* ── Объекты моторов ────────────────────────────────────────*/
static Motor_t motor_left;
static Motor_t motor_right;

/* ── RX ring buffer (заполняется из ISR через cdc_rx_hook) ──*/
#define RX_RING 256U
static uint8_t           s_rx[RX_RING];
static volatile uint32_t s_rx_head = 0U;
static volatile uint32_t s_rx_tail = 0U;

void cdc_rx_hook(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++) {
        uint32_t next = (s_rx_head + 1U) % RX_RING;
        if (next != s_rx_tail) {
            s_rx[s_rx_head] = buf[i];
            s_rx_head = next;
        }
    }
}

/* ── Команды скважности ─────────────────────────────────────*/
#define STEP_DUTY  0.1f
#define MAX_DUTY   1.0f

static float    s_cmd_left    = 0.0f;
static float    s_cmd_right   = 0.0f;
static uint32_t s_last_cmd_ms = 0U;

static void set_cmd(float l, float r)
{
    if (l >  MAX_DUTY) l =  MAX_DUTY;
    if (l < -MAX_DUTY) l = -MAX_DUTY;
    if (r >  MAX_DUTY) r =  MAX_DUTY;
    if (r < -MAX_DUTY) r = -MAX_DUTY;
    uint32_t pri = __get_PRIMASK();
    __disable_irq();
    s_cmd_left    = l;
    s_cmd_right   = r;
    s_last_cmd_ms = HAL_GetTick();
    __set_PRIMASK(pri);
}

/* ── USB CDC send ───────────────────────────────────────────*/
static void cdc_send(const char *s, uint16_t len)
{
    if (len == 0U || len > CDC_TX_BUF_SIZE) return;
    memcpy(UserTxBufferFS, s, len);
    uint32_t t = HAL_GetTick();
    while (CDC_Transmit_FS(UserTxBufferFS, len) == USBD_BUSY) {
        if (HAL_GetTick() - t > 30U) break;
    }
}

/* ── Консоль: WASD ──────────────────────────────────────────*/
static void handle_key(char c)
{
    uint32_t pri = __get_PRIMASK();
    __disable_irq();
    float l = s_cmd_left;
    float r = s_cmd_right;
    __set_PRIMASK(pri);

    switch (c) {
    case 'w': case 'W': l += STEP_DUTY; r += STEP_DUTY; break;
    case 's': case 'S': l -= STEP_DUTY; r -= STEP_DUTY; break;
    case 'a': case 'A': l -= STEP_DUTY; r += STEP_DUTY; break;
    case 'd': case 'D': l += STEP_DUTY; r -= STEP_DUTY; break;
    case ' ':           l = 0.0f;       r = 0.0f;       break;
    default: return;
    }
    set_cmd(l, r);

    char resp[48];
    int n = snprintf(resp, sizeof(resp), "cmd: L=%.2f R=%.2f\r\n",
                     (double)s_cmd_left, (double)s_cmd_right);
    if (n > 0) cdc_send(resp, (uint16_t)n);
}

/* ── Консоль: текстовые команды ─────────────────────────────*/
static void handle_line(const char *line)
{
    float vl = 0.0f, vr = 0.0f, v = 0.0f;

    if (strcmp(line, "stop") == 0) {
        set_cmd(0.0f, 0.0f);
        cdc_send("Stopped\r\n", 9U);
    } else if (sscanf(line, "left %f right %f", &vl, &vr) == 2) {
        set_cmd(vl, vr);
        char resp[48];
        int n = snprintf(resp, sizeof(resp), "cmd: L=%.2f R=%.2f\r\n",
                         (double)vl, (double)vr);
        if (n > 0) cdc_send(resp, (uint16_t)n);
    } else if (sscanf(line, "pid kp %f", &v) == 1) {
        motor_left.pid.kp  = v;
        motor_right.pid.kp = v;
        char resp[32]; int n = snprintf(resp, sizeof(resp), "kp=%.3f\r\n", (double)v);
        if (n > 0) cdc_send(resp, (uint16_t)n);
    } else if (sscanf(line, "pid ki %f", &v) == 1) {
        motor_left.pid.ki  = v;
        motor_right.pid.ki = v;
        char resp[32]; int n = snprintf(resp, sizeof(resp), "ki=%.3f\r\n", (double)v);
        if (n > 0) cdc_send(resp, (uint16_t)n);
    } else if (sscanf(line, "pid kd %f", &v) == 1) {
        motor_left.pid.kd  = v;
        motor_right.pid.kd = v;
        char resp[32]; int n = snprintf(resp, sizeof(resp), "kd=%.3f\r\n", (double)v);
        if (n > 0) cdc_send(resp, (uint16_t)n);
    } else if (strcmp(line, "help") == 0) {
        const char *h =
            "Команды:\r\n"
            "  w/s/a/d/space    — WASD управление\r\n"
            "  left V right V   — задать скважность [-1.0..1.0]\r\n"
            "  stop             — остановить\r\n"
            "  pid kp/ki/kd V   — настройка PID\r\n";
        cdc_send(h, (uint16_t)strlen(h));
    }
}

/* ── Консоль: неблокирующий drain RX буфера ─────────────────*/
static char    s_line[64];
static uint8_t s_line_len = 0U;

static void process_console(void)
{
    while (s_rx_tail != s_rx_head) {
        char c = (char)s_rx[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1U) % RX_RING;

        if (c == '\r' || c == '\n') {
            if (s_line_len > 0U) {
                s_line[s_line_len] = '\0';
                handle_line(s_line);
                s_line_len = 0U;
            }
        } else if (s_line_len == 0U &&
                   (c=='w' || c=='W' || c=='s' || c=='S' ||
                    c=='a' || c=='A' || c=='d' || c=='D' || c==' ')) {
            handle_key(c);
        } else {
            if (s_line_len < (uint8_t)(sizeof(s_line) - 1U)) {
                s_line[s_line_len++] = c;
            }
        }
    }
}

/* ── Прототипы инициализации ────────────────────────────────*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

void Error_Handler(void)
{
    GPIOE->BSRR = GPIO_PIN_1;
    while (1) {}
}

/* ── Точка входа ────────────────────────────────────────────*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();

    /* ── Инициализация левого мотора ────────────────────────*/
    motor_left.htim      = &htim2;
    motor_left.ch_fwd    = MOTOR_LEFT_CH_FWD;
    motor_left.ch_rev    = MOTOR_LEFT_CH_REV;
    motor_left.ccr_fwd   = &MOTOR_LEFT_CCR_FWD;
    motor_left.ccr_rev   = &MOTOR_LEFT_CCR_REV;
    motor_left.arr       = &TIM2->ARR;
    motor_left.en_port   = MOTOR_LEFT_EN_PORT;
    motor_left.en_r_pin  = MOTOR_LEFT_EN_R_PIN;
    motor_left.en_l_pin  = MOTOR_LEFT_EN_L_PIN;

    /* ── Инициализация правого мотора ───────────────────────*/
    motor_right.htim     = &htim2;
    motor_right.ch_fwd   = MOTOR_RIGHT_CH_FWD;
    motor_right.ch_rev   = MOTOR_RIGHT_CH_REV;
    motor_right.ccr_fwd  = &MOTOR_RIGHT_CCR_FWD;
    motor_right.ccr_rev  = &MOTOR_RIGHT_CCR_REV;
    motor_right.arr      = &TIM2->ARR;
    motor_right.en_port  = MOTOR_RIGHT_EN_PORT;
    motor_right.en_r_pin = MOTOR_RIGHT_EN_R_PIN;
    motor_right.en_l_pin = MOTOR_RIGHT_EN_L_PIN;

    motor_init(&motor_left,  0.01f);
    motor_init(&motor_right, 0.01f);

    MX_USB_DEVICE_Init();
    HAL_Delay(500U);

    const char *banner =
        "\r\n=== d-grape-test-motors: BTS7960B, PID плавности ===\r\n"
        "WASD / 'left V right V' / 'stop' / 'help'\r\n"
        "Скважность [-1.0 .. 1.0], шаг 0.1\r\n"
        "L_tgt   L_duty  R_tgt   R_duty\r\n"
        "-------------------------------------\r\n";
    cdc_send(banner, (uint16_t)strlen(banner));

    uint32_t last_pid   = HAL_GetTick() - 10U;
    uint32_t last_print = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();

        /* ── 100 Гц: PID обновление моторов ─────────────────*/
        if (now - last_pid >= 10U) {
            float dt = (float)(now - last_pid) / 1000.0f;
            last_pid = now;

            float cmd_l, cmd_r;
            uint32_t pri = __get_PRIMASK();
            __disable_irq();
            cmd_l = s_cmd_left;
            cmd_r = s_cmd_right;
            uint32_t age = now - s_last_cmd_ms;
            __set_PRIMASK(pri);

            /* Safety timeout 500 мс */
            if (age > CMD_TIMEOUT_MS) { cmd_l = 0.0f; cmd_r = 0.0f; }

            motor_set_target(&motor_left,  cmd_l);
            motor_set_target(&motor_right, cmd_r);
            motor_update(&motor_left,  dt);
            motor_update(&motor_right, dt);

            HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
        }

        /* ── Console input ───────────────────────────────────*/
        process_console();

        /* ── 5 Гц: вывод состояния ───────────────────────────*/
        if (now - last_print >= 200U) {
            last_print = now;
            char buf[80];
            int n = snprintf(buf, sizeof(buf),
                "%7.3f  %7.3f  %7.3f  %7.3f\r\n",
                (double)s_cmd_left,         (double)motor_left.duty_now,
                (double)s_cmd_right,        (double)motor_right.duty_now);
            if (n > 0) cdc_send(buf, (uint16_t)n);
        }
    }
}

/* ── Тактирование 168 МГц ───────────────────────────────────*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ── GPIO: LED PD12-15 + PE1 + EN PD0-PD3 ──────────────────*/
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    /* EN-пины + LED на GPIOD */
    gpio.Pin = MOTOR_LEFT_EN_R_PIN  | MOTOR_LEFT_EN_L_PIN  |
               MOTOR_RIGHT_EN_R_PIN | MOTOR_RIGHT_EN_L_PIN |
               GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* LED error на GPIOE */
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* Все LOW; EN будут выставлены HIGH в motor_init */
    HAL_GPIO_WritePin(GPIOD,
        MOTOR_LEFT_EN_R_PIN  | MOTOR_LEFT_EN_L_PIN  |
        MOTOR_RIGHT_EN_R_PIN | MOTOR_RIGHT_EN_L_PIN |
        GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
        GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
}

/* ── TIM2: PWM 20 кГц, 4 канала ────────────────────────────
 *  APB1 TIM clock = 84 МГц
 *  PSC=0, ARR=4199 → 84 000 000 / 4200 = 20 000 Гц
 *
 *  CH1 PA15 AF1 — L RPWM (вперёд)
 *  CH2 PB3  AF1 — R RPWM (вперёд)
 *  CH3 PB10 AF1 — L LPWM (назад)
 *  CH4 PB11 AF1 — R LPWM (назад)
 * ─────────────────────────────────────────────────────────── */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;

    /* PA15 — CH1 */
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB3 — CH2, PB10 — CH3, PB11 — CH4 */
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = 0;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 4199;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4);
}
