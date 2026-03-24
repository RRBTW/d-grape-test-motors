/* ============================================================
 *  main.c — d-grape-test-motors (BTS7960B x2, STM32F407G Discovery)
 *
 *  Разъёмы:
 *    Левый:   GND | PA15(RPWM) | 5V | PA7(LPWM)
 *    Правый:  GND | PB3(RPWM)  | 5V | PB11(LPWM)
 *
 *  Управление через USB CDC (крутит пока держишь):
 *    W  — вперёд       S  — назад
 *    A  — влево        D  — вправо
 *    пробел — стоп     stop / help (Enter)
 *
 *  Вывод 5 Гц:  L_tgt  L_duty  L_RPWM  L_LPWM  R_tgt  R_duty  R_RPWM  R_LPWM
 * ============================================================ */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "motors.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── HAL handles ─────────────────────────────────────────────*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* ── Объекты моторов ─────────────────────────────────────────*/
static Motor_t motor_left;
static Motor_t motor_right;

/* ── RX ring buffer ──────────────────────────────────────────*/
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

/* ── Команды ─────────────────────────────────────────────────
 *  Правый мотор стоит инвертированно: s_cmd_r хранит уже
 *  перевёрнутое значение (то, что реально пойдёт в драйвер).  */
static float    s_cmd_l       = 0.0f;
static float    s_cmd_r       = 0.0f;
static uint32_t s_last_cmd_ms = 0U;

static void set_drive(float l, float r)
{
    if (l >  1.0f) l =  1.0f;  if (l < -1.0f) l = -1.0f;
    if (r >  1.0f) r =  1.0f;  if (r < -1.0f) r = -1.0f;
    uint32_t pri = __get_PRIMASK();
    __disable_irq();
    s_cmd_l       = l;
    s_cmd_r       = -r;   /* инверт правого мотора */
    s_last_cmd_ms = HAL_GetTick();
    __set_PRIMASK(pri);
}

/* ── USB CDC send ────────────────────────────────────────────*/
static void cdc_send(const char *s, uint16_t len)
{
    if (len == 0U || len > CDC_TX_BUF_SIZE) return;
    memcpy(UserTxBufferFS, s, len);
    uint32_t t = HAL_GetTick();
    while (CDC_Transmit_FS(UserTxBufferFS, len) == USBD_BUSY) {
        if (HAL_GetTick() - t > 30U) break;
    }
}

/* ── Консоль: WASD (держи — едет, отпустил — стоп по таймауту)*/
static void handle_key(char c)
{
    switch (c) {
    case 'w': case 'W': set_drive(+1.0f, +1.0f); break;  /* вперёд  */
    case 's': case 'S': set_drive(-1.0f, -1.0f); break;  /* назад   */
    case 'a': case 'A': set_drive(-1.0f, +1.0f); break;  /* влево   */
    case 'd': case 'D': set_drive(+1.0f, -1.0f); break;  /* вправо  */
    case ' ':           set_drive( 0.0f,  0.0f); break;  /* стоп    */
    default: return;
    }
}

/* ── Консоль: текстовые команды ─────────────────────────────*/
static void handle_line(const char *line)
{
    if (strcmp(line, "stop") == 0) {
        set_drive(0.0f, 0.0f);
        cdc_send("Stopped\r\n", 9U);
    } else if (strcmp(line, "help") == 0) {
        const char *h =
            "Команды (держи клавишу — едет):\r\n"
            "  W/S/A/D  — вперёд/назад/влево/вправо\r\n"
            "  space    — стоп\r\n"
            "  stop     — стоп (Enter)\r\n"
            "  pid kp/ki/kd V — настройка PID\r\n";
        cdc_send(h, (uint16_t)strlen(h));
    } else {
        float v = 0.0f;
        if (sscanf(line, "pid kp %f", &v) == 1) {
            motor_left.pid.kp  = v;
            motor_right.pid.kp = v;
            char b[32]; int n = snprintf(b, sizeof(b), "kp=%.3f\r\n", (double)v);
            if (n > 0) cdc_send(b, (uint16_t)n);
        } else if (sscanf(line, "pid ki %f", &v) == 1) {
            motor_left.pid.ki  = v;
            motor_right.pid.ki = v;
            char b[32]; int n = snprintf(b, sizeof(b), "ki=%.3f\r\n", (double)v);
            if (n > 0) cdc_send(b, (uint16_t)n);
        } else if (sscanf(line, "pid kd %f", &v) == 1) {
            motor_left.pid.kd  = v;
            motor_right.pid.kd = v;
            char b[32]; int n = snprintf(b, sizeof(b), "kd=%.3f\r\n", (double)v);
            if (n > 0) cdc_send(b, (uint16_t)n);
        }
    }
}

/* ── Консоль: дрейн RX ───────────────────────────────────────*/
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
                   (c=='w'||c=='W'||c=='s'||c=='S'||
                    c=='a'||c=='A'||c=='d'||c=='D'||c==' ')) {
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
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void Error_Handler(void) { while (1) {} }

/* ── Точка входа ────────────────────────────────────────────*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();
    MX_TIM3_Init();

    /* ── Левый мотор: PA15(TIM2_CH1) RPWM, PA7(TIM3_CH2) LPWM ─*/
    motor_left.htim_fwd = &htim2;
    motor_left.ch_fwd   = MOTOR_LEFT_CH_FWD;
    motor_left.ccr_fwd  = &MOTOR_LEFT_CCR_FWD;
    motor_left.htim_rev = &htim3;
    motor_left.ch_rev   = MOTOR_LEFT_CH_REV;
    motor_left.ccr_rev  = &MOTOR_LEFT_CCR_REV;
    motor_left.arr      = &TIM2->ARR;

    /* ── Правый мотор: PB3(TIM2_CH2) RPWM, PB11(TIM2_CH4) LPWM */
    motor_right.htim_fwd = &htim2;
    motor_right.ch_fwd   = MOTOR_RIGHT_CH_FWD;
    motor_right.ccr_fwd  = &MOTOR_RIGHT_CCR_FWD;
    motor_right.htim_rev = &htim2;
    motor_right.ch_rev   = MOTOR_RIGHT_CH_REV;
    motor_right.ccr_rev  = &MOTOR_RIGHT_CCR_REV;
    motor_right.arr      = &TIM2->ARR;

    motor_init(&motor_left,  0.01f);
    motor_init(&motor_right, 0.01f);

    MX_USB_DEVICE_Init();
    HAL_Delay(500U);

    const char *banner =
        "\r\n=== d-grape-test-motors: BTS7960B x2, STM32F407G Discovery ===\r\n"
        "Держи W/S/A/D — едет; отпустил — стоп. space/stop — немедленный стоп.\r\n"
        " L_tgt   L_duty  L_RPWM L_LPWM   R_tgt   R_duty  R_RPWM R_LPWM\r\n"
        "-------------------------------------------------------------------\r\n";
    cdc_send(banner, (uint16_t)strlen(banner));

    uint32_t last_pid   = HAL_GetTick() - 10U;
    uint32_t last_print = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();

        /* ── 100 Гц: PID ─────────────────────────────────────*/
        if (now - last_pid >= 10U) {
            float dt = (float)(now - last_pid) / 1000.0f;
            last_pid = now;

            float cmd_l, cmd_r;
            uint32_t pri = __get_PRIMASK();
            __disable_irq();
            cmd_l = s_cmd_l;
            cmd_r = s_cmd_r;
            uint32_t age = now - s_last_cmd_ms;
            __set_PRIMASK(pri);

            /* Стоп если клавиша отпущена (нет обновлений) */
            if (age > CMD_TIMEOUT_MS) { cmd_l = 0.0f; cmd_r = 0.0f; }

            motor_set_target(&motor_left,  cmd_l);
            motor_set_target(&motor_right, cmd_r);
            motor_update(&motor_left,  dt);
            motor_update(&motor_right, dt);
        }

        process_console();

        /* ── 5 Гц: вывод состояния + CCR ────────────────────*/
        if (now - last_print >= 200U) {
            last_print = now;
            char buf[96];
            int n = snprintf(buf, sizeof(buf),
                "%7.3f  %7.3f  %5lu  %5lu   %7.3f  %7.3f  %5lu  %5lu\r\n",
                (double)s_cmd_l,          (double)motor_left.duty_now,
                (unsigned long)TIM2->CCR1, (unsigned long)TIM3->CCR2,
                (double)s_cmd_r,          (double)motor_right.duty_now,
                (unsigned long)TIM2->CCR2, (unsigned long)TIM2->CCR4);
            if (n > 0) cdc_send(buf, (uint16_t)n);
        }
    }
}

/* ── Тактирование 168 МГц (STM32F407, HSE 8 МГц) ───────────
 *  VCO = 8/8 * 336 = 336 МГц
 *  SYSCLK = 336/2 = 168 МГц
 *  USB    = 336/7 = 48 МГц
 *  APB1 TIM clock = 42*2 = 84 МГц → PSC=0, ARR=4199 → 20 кГц
 * ─────────────────────────────────────────────────────────── */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;  /* 336/2 = 168 МГц */
    osc.PLL.PLLQ       = 7;              /* 336/7 = 48 МГц USB */
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;  /* APB1 = 42 МГц, TIM = 84 МГц */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ── TIM2: 4 канала PWM, 20 кГц ─────────────────────────────
 *  CH1 PA15 AF1 — левый  RPWM
 *  CH2 PB3  AF1 — правый RPWM
 *  CH4 PB11 AF1 — правый LPWM
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

    gpio.Pin = GPIO_PIN_15;                    /* PA15 — CH1 */
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_11;       /* PB3 — CH2, PB11 — CH4 */
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
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4);

    /* Включаем каналы и таймер напрямую через регистры —
     * HAL_TIM_PWM_Start блокирует handle после первого вызова
     * и CH2/CH4 не стартуют на том же htim. */
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E;
    TIM2->CR1  |= TIM_CR1_CEN;
}

/* ── TIM3_CH2: левый LPWM → PA7 (AF2), 20 кГц ─────────────*/
static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* GPIOA уже включён */

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = 0;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = 4199;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
