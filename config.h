#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
static inline void Security_EnableReadProtection(void);
/* ============================================================
 * 1. CLOCK & FREQUENCY DEFINITIONS
 * ============================================================ */
#define SYSCLK_FREQ_HZ              72000000UL
#define APB1_FREQ_HZ                36000000UL
#define APB2_FREQ_HZ                72000000UL
#define TIM_PRESCALER_1US           71U         /* 72MHz / (71 + 1) = 1MHz -> 1us resolution */

/* ============================================================
 * 2. CRANK & TRIGGER WHEEL (60-2)
 * ============================================================ */
#define CRANK_TOTAL_TEETH           60U
#define CRANK_MISSING_TEETH         2U
#define CRANK_ACTIVE_TEETH          58U
#define CRANK_DEGREES_PER_TOOTH     6U          /* 360 deg / 60 teeth = 6 deg */
#define CRANK_GAP_RATIO_MIN_X100    180U        /* 1.8x previous period -> Missing tooth detected */
#define CRANK_GAP_RATIO_MAX_X100    320U        /* 3.2x previous period */
#define CRANK_BASE_OFFSET_DEG       120U        /* Trigger offset: 120 deg (20 teeth before TDC) */

/* ============================================================
 * 3. IGNITION PARAMETERS (4-CYLINDER WASTED SPARK)
 * ============================================================ */
#define DEFAULT_BASE_ADVANCE_DEG    10U         /* 10 deg BTDC */
#define DEFAULT_DWELL_TIME_US       3000U       /* 3.0 ms typical coil dwell */
#define MIN_DWELL_TIME_US           1500U       /* 1.5 ms */
#define MAX_DWELL_TIME_US           5000U       /* 5.0 ms */
#define MAX_ADVANCE_DEG             45U
#define MIN_ADVANCE_DEG             0U

/* ============================================================
 * 4. GOVERNOR & ENGINE SPECS (GENERATOR TUNING & SOFT START)
 * ============================================================ */
#define CRANK_EXIT_RPM_THRESHOLD    450U        /* Crank-to-Run detection threshold */
#define WARMUP_IDLE_RPM             850U        /* Warm-up / Idle RPM */
#define WARMUP_HOLD_TIME_MS         3000U       /* Time to stay at warm-up RPM (3.0s) */
#define RAMP_UP_TIME_MS             4000U       /* Ramp time from idle to nominal RPM (4.0s) */
#define GOVERNOR_TARGET_RPM         1500U       /* Target generator locked RPM (50Hz) */
#define ENGINE_MAX_RPM              3600U       /* Absolute safe limit */
#define ENGINE_OVERSPEED_RPM        3800U       /* Instant trip cut-off */

#define THROTTLE_PWM_FREQ_HZ        20000UL     /* 20kHz switching for smooth DC control */
#define THROTTLE_MAX_DUTY           1000U       /* 100.0% max PWM */
#define THROTTLE_MIN_DUTY           0U          /* 0.0% min PWM */

/* ============================================================
 * 5. MODBUS RTU & UART1
 * ============================================================ */
#define MODBUS_SLAVE_ADDRESS        1U
#define MODBUS_BAUDRATE             115200UL
#define MODBUS_RX_BUF_SIZE          64U
#define MODBUS_TX_BUF_SIZE          64U
#define MODBUS_HOLDING_REGS_COUNT   32U

/* ============================================================
 * 6. PIN & PORT MAPPING
 * ============================================================ */
/* Crank Input (TIM2_CH1) */
#define CRANK_PORT                  GPIOA
#define CRANK_PIN_BIT               0U          /* PA0 */

/* Dual Ignition Coils (TIM3 CH1 & CH2) */
#define IGN_COIL_A_PORT             GPIOA
#define IGN_COIL_A_PIN_BIT          6U          /* PA6 = TIM3_CH1 (Cylinders 1 & 4) */
#define IGN_COIL_B_PORT             GPIOA
#define IGN_COIL_B_PIN_BIT          7U          /* PA7 = TIM3_CH2 (Cylinders 2 & 3) */

/* Throttle H-Bridge PWM (TIM4 CH1 & CH2) & Driver Enable */
#define THROTTLE_PWM_PORT           GPIOB
#define THROTTLE_PWM_PIN1_BIT       6U          /* PB6 = TIM4_CH1 (Forward/Open) */
#define THROTTLE_PWM_PIN2_BIT       7U          /* PB7 = TIM4_CH2 (Reverse/Close) */
#define THROTTLE_EN_PORT            GPIOB
#define THROTTLE_EN_PIN_BIT         5U          /* PB5 = H-Bridge Enable (Active High) */

/* Dual TPS Analog Inputs (ADC1) */
#define TPS1_ADC_CHANNEL            1U          /* PA1 = ADC1_IN1 (TPS Main) */
#define TPS2_ADC_CHANNEL            5U          /* PA5 = ADC1_IN5 (TPS Secondary/Safety) */

/* TPS Calibration & Limits (12-bit ADC: 0 - 4095) */
#define TPS1_MIN_RAW_DEFAULT        620U        /* ~0.5V (0.0% Closed Position) */
#define TPS1_MAX_RAW_DEFAULT        3720U       /* ~4.5V (100.0% Fully Open) */

/* TPS Plausibility & Out-of-Range Safety Limits */
#define TPS_SUM_EXPECTED_RAW        4095U       /* 12-bit ADC full range */
#define TPS_PLAUSIBILITY_TOL_RAW    250U        /* Max allowed deviation (~6%) */
#define TPS_SHORT_GND_THRESHOLD     200U        /* Below ~0.16V -> Short to GND / Disconnected */
#define TPS_SHORT_VCC_THRESHOLD     3950U       /* Above ~3.18V -> Short to VREF */

/* RS485 Direction Pin */
#define RS485_DIR_PORT              GPIOA
#define RS485_DIR_PIN_BIT           8U          /* PA8 */

/* Relays & Outputs */
#define FUEL_RELAY_PORT             GPIOB
#define FUEL_RELAY_PIN_BIT          0U          /* PB0 */
#define STARTER_RELAY_PORT          GPIOB
#define STARTER_RELAY_PIN_BIT       1U          /* PB1 */

/* ??????? ?????????? ????? ???? ??? ? ???? ??????? ??? ????? ????? STM32 */
#define FLASH_KEY1_VAL      ((uint32_t)0x45670123)
#define FLASH_KEY2_VAL      ((uint32_t)0xCDEF89AB)



static inline void Security_EnableReadProtection(void)
{
    if ((FLASH->OBR & FLASH_OBR_RDPRT) != 0) {
        return; /* ????? ???? ??? ??? */
    }

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->OPTKEYR = 0x45670123;
    FLASH->OPTKEYR = 0xCDEF89AB;

    while (FLASH->SR & FLASH_SR_BSY);

    FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= (uint32_t)(~FLASH_CR_OPTER);

    FLASH->CR |= FLASH_CR_OPTPG;
    OB->RDP = 0x00; /* ???? ???? ??? 1 */
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= (uint32_t)(~FLASH_CR_OPTPG);

    FLASH->CR |= FLASH_CR_LOCK;

    /* ???? ????? ??? ????? Option Bytes */
    NVIC_SystemReset();
}
#endif /* CONFIG_H_ */