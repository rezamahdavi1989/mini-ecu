
/* ============================================================================
 * main.c — Mini-ECU (STM32F103C8T6) — Reza Mahdavi — Production 3.0
 * Register-level / Bare-metal —  HAL
 * ========================================================================== */
#include "stm32f10x.h"
#include "config.h"
#include "ecu_types.h"
#include "hmi.h"

/* ============================ Global Data ================================= */
 static volatile ECU_Runtime_t g_ecu;
 static volatile uint16_t g_holding_regs[MODBUS_HOLDING_REGS_COUNT];

/* ============================ Modbus Constants & Table ==================== */
#define MB_T35_MS          2u
#define MB_FC_READ_HOLD    0x03u
#define MB_FC_READ_INPUT   0x04u
#define MB_FC_WRITE_SINGLE 0x06u
#define MB_FC_WRITE_MULTI  0x10u

/*  Fast CRC-16 Modbus (Poly 0xA001)  Flash */
static const uint16_t g_mb_crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static volatile uint8_t  mb_rx_buf[MODBUS_RX_BUF_SIZE];
static volatile uint8_t  mb_rx_len     = 0;
static volatile uint8_t  mb_frame_rdy  = 0;
static volatile uint16_t mb_idle_ms    = 0;


static volatile uint8_t  tick_1ms_flag   = 0;
static volatile uint8_t  tick_10ms_flag  = 0;
static volatile uint8_t  tick_100ms_flag = 0;


static uint32_t crank_last_capture = 0;
static uint32_t crank_silent_ms    = 0;
static int32_t  gov_integral       = 0;


/* ============================ Prototypes ================================== */
static void SystemClock_72MHz(void);
static void GPIO_Init(void);
static void TIM2_Crank_Init(void);
static void TIM3_Ignition_Init(void);
static void TIM4_Throttle_Init(void);
static void ADC1_Init(void);
static void USART1_Init(void);
static void SysTick_Init(void);
static void MB_ProcessFrame(void);
static uint16_t MB_CRC16(const uint8_t *p, uint16_t len);
static void MB_Send(const uint8_t *d, uint16_t len);
static void Governor_Task(void);
static void Safety_Task(void);
static void Throttle_Drive(void);
static void ADC_Sample(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void Lock_Flash_Protection(void);
/* ============================================================================
 * main
 * ========================================================================== */







int main(void)
{
    SystemClock_72MHz();
    GPIO_Init();
    TIM2_Crank_Init();
    TIM3_Ignition_Init();
    TIM4_Throttle_Init();
    ADC1_Init();
    USART1_Init();
    SysTick_Init();
    HMI_Init();
    Security_EnableReadProtection();


    g_ecu.system_state = ECU_STATE_STANDBY;
    g_ecu.fault_flags  = ECU_FAULT_NONE;
    g_ecu.uptime_ms    = 0u;
    g_ecu.engine_rpm   = 0u;

    g_ecu.crank.sync_state = CRANK_SYNC_LOST;
    g_ecu.crank.current_tooth = 0;

    g_ecu.ignition.advance_deg_x10   = (uint16_t)(DEFAULT_BASE_ADVANCE_DEG * 10);
    g_ecu.ignition.dwell_time_us     = DEFAULT_DWELL_TIME_US;
    g_ecu.ignition.fire_tooth_coil_a = (CRANK_BASE_OFFSET_DEG / CRANK_DEGREES_PER_TOOTH) - (DEFAULT_BASE_ADVANCE_DEG / CRANK_DEGREES_PER_TOOTH);
    g_ecu.ignition.fire_tooth_coil_b = g_ecu.ignition.fire_tooth_coil_a + (CRANK_TOTAL_TEETH / 2U);
    g_ecu.ignition.next_coil         = COIL_NONE;
    g_ecu.ignition.spark_armed       = 0u;
    g_ecu.ignition.dwell_active      = false;

    g_ecu.governor.target_tps_raw    = 0u;
    g_ecu.governor.actual_tps1_raw   = 0u;
    g_ecu.governor.actual_tps2_raw   = 0u;
    g_ecu.governor.tps_health        = TPS_STATUS_OK;
    g_ecu.governor.pwm_duty          = 0;
    g_ecu.governor.hbridge_enabled   = false;
    g_ecu.governor.target_rpm        = GOVERNOR_TARGET_RPM;

    HMI_SyncRegisters();
    g_holding_regs[HMI_REG_TARGET_RPM] = GOVERNOR_TARGET_RPM;

    __enable_irq();

    for (;;)
    {

        if (tick_1ms_flag) {
            tick_1ms_flag = 0;
            g_ecu.uptime_ms++;
            if (crank_silent_ms < 0xFFFFFFFFUL) crank_silent_ms++;
        }


        if (tick_10ms_flag) {
            tick_10ms_flag = 0;
            ADC_Sample();
            Governor_Task();
            Safety_Task();
            Throttle_Drive();
            HMI_UpdateLED(g_ecu.system_state);
        }


        if (tick_100ms_flag) {
            tick_100ms_flag = 0;
            HMI_SyncRegisters();
        }

        /*  RTU */
        if (mb_frame_rdy) {
            __disable_irq();
            mb_frame_rdy = 0;
            __enable_irq();
            MB_ProcessFrame();
        }
    }
}

/* ============================================================================
 * 72— HSE 8MHz × 9 = 72MHz
 * ========================================================================== */
static void SystemClock_72MHz(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) { }

    FLASH->ACR |= FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
    RCC->CFGR  |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR    |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
    SystemCoreClock = SYSCLK_FREQ_HZ;
}

/* ============================================================================
 * (GPIO Register Level)
 * ========================================================================== */
static void GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN
                  | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    /* PA0 —  (Input Pull-Up) */
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOA->CRL |=  GPIO_CRL_CNF0_1;
    GPIOA->ODR |=  GPIO_ODR_ODR0;

    /* PA1 — TPS1 */
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
    /* PA5 — TPS2 */
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);

    /* PA6 & PA7 —  A  B (Push-Pull 50MHz) */
    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6
                  | GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
    GPIOA->CRL |=  (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0)
                 | (GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0);
    GPIOA->BRR  =  (1u << IGN_COIL_A_PIN_BIT) | (1u << IGN_COIL_B_PIN_BIT);

    /* PB5 —  (Output Push-Pull) */
    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOB->CRL |=  GPIO_CRL_MODE5_1;
    GPIOB->BRR  =  (1u << THROTTLE_EN_PIN_BIT);

    /* PB6 & PB7 —  PWM  (TIM4 CH1 & CH2 AF Push-Pull) */
    GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6
                  | GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
    GPIOB->CRL |=  GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_1
                 | GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_1;

    /* PA8 —  RS485 (Output Push-Pull) */
    GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    GPIOA->CRH |=  GPIO_CRH_MODE8_1;
    GPIOA->BRR  =  (1u << RS485_DIR_PIN_BIT);

    /* PA9 — USART1 TX (AF Push-Pull) */
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |=  GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;
    /* PA10 — USART1 RX (Input Pull-Up) */
    GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
    GPIOA->CRH |=  GPIO_CRH_CNF10_1;
    GPIOA->ODR |=  GPIO_ODR_ODR10;

    /* PC13 —  LED  */
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |=  GPIO_CRH_MODE13_1;
    GPIOC->BSRR =  GPIO_BSRR_BS13;
}

/* ============================================================================
 *  —  (Input Capture)
 * ========================================================================== */
static void TIM2_Crank_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = TIM_PRESCALER_1US;
    TIM2->ARR = 0xFFFFu;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCER  &= ~TIM_CCER_CC1P;
    TIM2->CCER  |=  TIM_CCER_CC1E;
    TIM2->DIER  |=  TIM_DIER_CC1IE;
    TIM2->CR1   |=  TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
}

/* ============================================================================
 *  (Output Compare)
 * ========================================================================== */
static void TIM3_Ignition_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = TIM_PRESCALER_1US;
    TIM3->ARR = 0xFFFFu;
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_2;
    TIM3->CCER  &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM3->DIER  |=  TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM3->CR1   |=  TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);
}

/* ============================================================================
 *  (PWM 20kHz)
 * ========================================================================== */
static void TIM4_Throttle_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 0u;
    TIM4->ARR = (APB1_FREQ_HZ * 2UL / THROTTLE_PWM_FREQ_HZ) - 1u;
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1
                 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4->CCR1 = 0u;
    TIM4->CCR2 = 0u;
    TIM4->CR1  |= TIM_CR1_CEN;
}

/* ============================================================================
 *(ADC1)
 * ========================================================================== */
static void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SMPR2 |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP5_2;
    ADC1->CR2   |= ADC_CR2_ADON;
    ADC1->CR2   |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) { }
}

static uint16_t ADC1_ReadChannel(uint8_t ch)
{
    ADC1->SQR3 = ch;
    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC)) { }
    return (uint16_t)ADC1->DR;
}

static void ADC_Sample(void)
{
    uint16_t raw1 = ADC1_ReadChannel(TPS1_ADC_CHANNEL);
    uint16_t raw2 = ADC1_ReadChannel(TPS2_ADC_CHANNEL);

    g_ecu.governor.actual_tps1_raw = raw1;
    g_ecu.governor.actual_tps2_raw = raw2;

    if (raw1 < TPS_SHORT_GND_THRESHOLD || raw2 < TPS_SHORT_GND_THRESHOLD) {
        g_ecu.governor.tps_health = TPS_STATUS_ERR_SHORT_GND;
        g_ecu.fault_flags |= ECU_FAULT_TPS_PLAUSIBILITY;
    } else if (raw1 > TPS_SHORT_VCC_THRESHOLD || raw2 > TPS_SHORT_VCC_THRESHOLD) {
        g_ecu.governor.tps_health = TPS_STATUS_ERR_SHORT_VCC;
        g_ecu.fault_flags |= ECU_FAULT_TPS_PLAUSIBILITY;
    } else {
        uint32_t sum = (uint32_t)raw1 + (uint32_t)raw2;
        int32_t diff = (int32_t)sum - (int32_t)TPS_SUM_EXPECTED_RAW;
        if (diff < 0) diff = -diff;

        if (diff > (int32_t)TPS_PLAUSIBILITY_TOL_RAW) {
            g_ecu.governor.tps_health = TPS_STATUS_ERR_MISMATCH;
            g_ecu.fault_flags |= ECU_FAULT_TPS_PLAUSIBILITY;
        } else {
            g_ecu.governor.tps_health = TPS_STATUS_OK;
        }
    }
}

/* ============================================================================
 *  RS485  Modbus RTU
 * ========================================================================== */
static void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = (APB2_FREQ_HZ + (MODBUS_BAUDRATE / 2u)) / MODBUS_BAUDRATE;
    USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 3);
}

static uint16_t MB_CRC16(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc = (uint16_t)((crc >> 8) ^ g_mb_crc_table[(crc ^ *p++) & 0xFFu]);
    }
    return crc;
}

static void MB_Send(const uint8_t *d, uint16_t len)
{
    GPIOA->BSRR = (1u << RS485_DIR_PIN_BIT); /* DE High = ??????? */
    for (uint16_t i = 0; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE)) { }
        USART1->DR = d[i];
    }
    while (!(USART1->SR & USART_SR_TC)) { }
    GPIOA->BRR  = (1u << RS485_DIR_PIN_BIT); /* RE Low = ?????? */
    (void)USART1->SR;
    (void)USART1->DR;
}

static void MB_ProcessFrame(void)
{
    uint8_t  *b  = (uint8_t *)mb_rx_buf;
    uint16_t len = mb_rx_len;
    uint8_t  rsp[MODBUS_TX_BUF_SIZE];
    uint16_t rlen = 0u;

    if (len < 4u || b[0] != MODBUS_SLAVE_ADDRESS) return;
    if (MB_CRC16(b, len - 2u) != (uint16_t)(b[len - 2u] | (b[len - 1u] << 8))) return;

    uint8_t  fc    = b[1];
    uint16_t addr  = (uint16_t)((b[2] << 8) | b[3]);
    uint16_t count = (uint16_t)((b[4] << 8) | b[5]);

    switch (fc)
    {
    case MB_FC_READ_HOLD:
    case MB_FC_READ_INPUT:
    {
        if (count < 1u || count > 32u) return;
        if ((uint32_t)addr + count > MODBUS_HOLDING_REGS_COUNT) return;

        rsp[rlen++] = MODBUS_SLAVE_ADDRESS;
        rsp[rlen++] = fc;
        rsp[rlen++] = (uint8_t)(count * 2u);

        for (uint16_t i = 0; i < count; i++) {
            uint16_t v = HMI_GetRegister((uint16_t)(addr + i));
            rsp[rlen++] = (uint8_t)(v >> 8);
            rsp[rlen++] = (uint8_t)(v & 0xFFu);
        }
        break;
    }

    case MB_FC_WRITE_SINGLE:
    {
        if (addr >= MODBUS_HOLDING_REGS_COUNT) return;
        HMI_SetRegister(addr, count);
        HMI_SyncRegisters();

        rsp[rlen++] = MODBUS_SLAVE_ADDRESS;
        rsp[rlen++] = fc;
        rsp[rlen++] = b[2];
        rsp[rlen++] = b[3];
        rsp[rlen++] = b[4];
        rsp[rlen++] = b[5];
        break;
    }

    case MB_FC_WRITE_MULTI:
    {
        uint8_t byte_count = b[6];
        if (count < 1u || count > 32u) return;
        if ((uint32_t)addr + count > MODBUS_HOLDING_REGS_COUNT) return;
        if (byte_count != (count * 2u)) return;

        for (uint16_t i = 0; i < count; i++) {
            uint16_t v = (uint16_t)((b[7 + i * 2u] << 8) | b[8 + i * 2u]);
            HMI_SetRegister((uint16_t)(addr + i), v);
        }
        HMI_SyncRegisters();

        rsp[rlen++] = MODBUS_SLAVE_ADDRESS;
        rsp[rlen++] = fc;
        rsp[rlen++] = b[2];
        rsp[rlen++] = b[3];
        rsp[rlen++] = b[4];
        rsp[rlen++] = b[5];
        break;
    }

    default:
        return;
    }

    uint16_t crc = MB_CRC16(rsp, rlen);
    rsp[rlen++] = (uint8_t)(crc & 0xFFu);
    rsp[rlen++] = (uint8_t)(crc >> 8);
    MB_Send(rsp, rlen);
}

/* ============================================================================
 *  (SysTick)
 * ========================================================================== */
static void SysTick_Init(void)
{
    SysTick->LOAD = (SYSCLK_FREQ_HZ / 1000u) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    static uint8_t cnt10 = 0u, cnt100 = 0u;

    tick_1ms_flag = 1u;
    if (++cnt10  >= 10u)  { cnt10  = 0u; tick_10ms_flag  = 1u; }
    if (++cnt100 >= 100u) { cnt100 = 0u; tick_100ms_flag = 1u; }

    if (mb_rx_len > 0u && !mb_frame_rdy) {
        if (++mb_idle_ms >= MB_T35_MS) {
            mb_frame_rdy = 1u;
            mb_idle_ms   = 0u;
        }
    }
}

/* ============================================================================
 *  USART1
 * ========================================================================== */
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)
    {
        uint8_t c = (uint8_t)USART1->DR;
        if (mb_rx_len < MODBUS_RX_BUF_SIZE) {
            mb_rx_buf[mb_rx_len++] = c;
        }
        mb_idle_ms = 0u;
    }
}

/* ============================================================================
 * TIM2 —
 * ========================================================================== */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF)
    {
        uint32_t now    = TIM2->CCR1;
        uint32_t period = (uint16_t)(now - crank_last_capture);
        crank_last_capture = now;
        crank_silent_ms    = 0u;

        g_ecu.crank.tooth_period_us = period;
        g_ecu.crank.tooth_counter_raw++;

        uint32_t avg = g_ecu.crank.avg_tooth_period_us;

        /* (Ratio > 1.8x) */
        if (avg > 0u && (period * 100u) > (avg * CRANK_GAP_RATIO_MIN_X100)) {
            g_ecu.crank.current_tooth = 1u;
            g_ecu.crank.sync_state    = CRANK_SYNC_LOCKED;
            g_ecu.engine_rpm          = (uint16_t)((60000000UL / CRANK_TOTAL_TEETH) / avg);
        } else {
            /*  */
            if (g_ecu.crank.avg_tooth_period_us == 0u) {
                g_ecu.crank.avg_tooth_period_us = period;
            } else {
                g_ecu.crank.avg_tooth_period_us = (g_ecu.crank.avg_tooth_period_us * 3u + period) >> 2;
            }

            if (g_ecu.crank.current_tooth < CRANK_ACTIVE_TEETH) {
                g_ecu.crank.current_tooth++;
            }
        }

        /* */
        if ((g_ecu.system_state == ECU_STATE_RUNNING || g_ecu.system_state == ECU_STATE_GOVERNOR_ACTIVE)
            && g_ecu.crank.sync_state == CRANK_SYNC_LOCKED && avg > 0u)
        {
            uint32_t dwell_us = g_ecu.ignition.dwell_time_us;
            uint32_t teeth_dwell = (dwell_us + avg - 1u) / avg;
            if (teeth_dwell == 0u) teeth_dwell = 1u;

            uint32_t fire_a = g_ecu.ignition.fire_tooth_coil_a;
            uint32_t fire_b = g_ecu.ignition.fire_tooth_coil_b;
            uint32_t start_a = (fire_a > teeth_dwell) ? (fire_a - teeth_dwell) : 1u;
            uint32_t start_b = (fire_b > teeth_dwell) ? (fire_b - teeth_dwell) : 1u;

            if (g_ecu.crank.current_tooth == start_a && !g_ecu.ignition.dwell_active) {
                GPIOA->BSRR = (1u << IGN_COIL_A_PIN_BIT); /* ???? ???? ???? A */
                g_ecu.ignition.dwell_active = true;
                g_ecu.ignition.next_coil    = COIL_A;
                TIM3->CCR1 = (uint16_t)(TIM3->CNT + dwell_us);
                TIM3->SR   &= ~TIM_SR_CC1IF;
                TIM3->CCER |=  TIM_CCER_CC1E;
            }

            if (g_ecu.crank.current_tooth == start_b && !g_ecu.ignition.dwell_active) {
                GPIOA->BSRR = (1u << IGN_COIL_B_PIN_BIT); /* ???? ???? ???? B */
                g_ecu.ignition.dwell_active = true;
                g_ecu.ignition.next_coil    = COIL_B;
                TIM3->CCR2 = (uint16_t)(TIM3->CNT + dwell_us);
                TIM3->SR   &= ~TIM_SR_CC2IF;
                TIM3->CCER |=  TIM_CCER_CC2E;
            }
        }

        TIM2->SR = 0u;
    }
}

/* ============================================================================
 *  TIM3 —  (Spark Discharge)
 * ========================================================================== */
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_CC1IF) {
        GPIOA->BRR = (1u << IGN_COIL_A_PIN_BIT); /* ??? ???? A -> ???? */
        TIM3->CCER &= ~TIM_CCER_CC1E;
        TIM3->SR   &= ~TIM_SR_CC1IF;
        g_ecu.ignition.dwell_active = false;
    }
    if (TIM3->SR & TIM_SR_CC2IF) {
        GPIOA->BRR = (1u << IGN_COIL_B_PIN_BIT); /* ??? ???? B -> ???? */
        TIM3->CCER &= ~TIM_CCER_CC2E;
        TIM3->SR   &= ~TIM_SR_CC2IF;
        g_ecu.ignition.dwell_active = false;
    }
}

/* ============================================================================
 *  PID
 * ========================================================================== */
static void Governor_Task(void)
{
    if (g_ecu.fault_flags != ECU_FAULT_NONE) {
        g_ecu.system_state = ECU_STATE_FAULT_SAFE_STOP;
        g_ecu.governor.pwm_duty = 0;
        return;
    }

    if (g_ecu.engine_rpm >= CRANK_EXIT_RPM_THRESHOLD && g_ecu.system_state == ECU_STATE_STANDBY) {
        g_ecu.system_state = ECU_STATE_RUNNING;
    }

    if (g_ecu.system_state != ECU_STATE_RUNNING && g_ecu.system_state != ECU_STATE_GOVERNOR_ACTIVE) {
        g_ecu.governor.pwm_duty = 0;
        gov_integral = 0;

        return;
    }

    if (g_ecu.governor.tps_health != TPS_STATUS_OK) return;

    int32_t target  = (int32_t)g_holding_regs[HMI_REG_TARGET_RPM];
    int32_t current = (int32_t)g_ecu.engine_rpm;
    int32_t err     = target - current;

    gov_integral += err;
    if (gov_integral > 10000)  gov_integral = 10000;
    if (gov_integral < -10000) gov_integral = -10000;

    int32_t kp = (int32_t)g_holding_regs[HMI_REG_KP_X1000];
    int32_t ki = (int32_t)g_holding_regs[HMI_REG_KI_X1000];

    int32_t out = (kp * err + ki * gov_integral ) / 1000;

    if (out > (int32_t)THROTTLE_MAX_DUTY)  out = (int32_t)THROTTLE_MAX_DUTY;
    if (out < -(int32_t)THROTTLE_MAX_DUTY) out = -(int32_t)THROTTLE_MAX_DUTY;

    g_ecu.governor.pwm_duty = (int16_t)out;
}

/* ============================================================================
 *  H-Bridge
 * ========================================================================== */
static void Throttle_Drive(void)
{
    int32_t duty = g_ecu.governor.pwm_duty;
    uint32_t max_period = (APB1_FREQ_HZ * 2UL / THROTTLE_PWM_FREQ_HZ);

    if (duty > 0) {
        uint32_t ticks = ((uint32_t)duty * max_period) / 1000u;
        TIM4->CCR1 = ticks;
        TIM4->CCR2 = 0u;
        GPIOB->BSRR = (1u << THROTTLE_EN_PIN_BIT);
        g_ecu.governor.hbridge_enabled = true;
    }
    else if (duty < 0) {
        uint32_t d = (uint32_t)(-duty);
        uint32_t ticks = (d * max_period) / 1000u;
        TIM4->CCR1 = 0u;
        TIM4->CCR2 = ticks;
        GPIOB->BSRR = (1u << THROTTLE_EN_PIN_BIT);
        g_ecu.governor.hbridge_enabled = true;
    }
    else {
        TIM4->CCR1 = 0u;
        TIM4->CCR2 = 0u;
        GPIOB->BRR  = (1u << THROTTLE_EN_PIN_BIT);
        g_ecu.governor.hbridge_enabled = false;
    }
}

/* ============================================================================
 *  Interlock
 * ========================================================================== */
static void Safety_Task(void)
{
    if (g_ecu.engine_rpm > ENGINE_OVERSPEED_RPM) {
        g_ecu.fault_flags |= ECU_FAULT_OVERSPEED;
    }

    if (crank_silent_ms > 200u && (g_ecu.system_state == ECU_STATE_RUNNING || g_ecu.system_state == ECU_STATE_GOVERNOR_ACTIVE)) {
        g_ecu.fault_flags |= ECU_FAULT_CRANK_LOSS;
    }

    if (g_ecu.fault_flags != ECU_FAULT_NONE)
    {
        g_ecu.system_state = ECU_STATE_FAULT_SAFE_STOP;

        /*  */
        GPIOA->BRR = (1u << IGN_COIL_A_PIN_BIT) | (1u << IGN_COIL_B_PIN_BIT);
        g_ecu.ignition.dwell_active = false;
        g_ecu.ignition.spark_armed  = 0u;
        TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);

        /*  */
        g_ecu.governor.pwm_duty = 0;
        TIM4->CCR1 = 0u;
        TIM4->CCR2 = 0u;
        GPIOB->BRR  = (1u << THROTTLE_EN_PIN_BIT);
        g_ecu.governor.hbridge_enabled = false;
    }
}

/* ============================================================================
 *  HMI
 * ========================================================================== */
void HMI_Init(void)
{
    for (uint8_t i = 0; i < MODBUS_HOLDING_REGS_COUNT; i++) {
        g_holding_regs[i] = 0;
    }
    g_holding_regs[HMI_REG_KP_X1000] = 1200u;
    g_holding_regs[HMI_REG_KI_X1000] = 50u;
    g_holding_regs[HMI_REG_KD_X1000] = 300u;
    g_holding_regs[HMI_REG_TARGET_RPM] = GOVERNOR_TARGET_RPM;
}

void HMI_UpdateLED(ECU_State_t state)
{
    static uint16_t led_cnt = 0;
    led_cnt++;

    switch (state)
    {
    case ECU_STATE_STANDBY:
        if (led_cnt % 100 == 0) GPIOC->ODR ^= GPIO_ODR_ODR13;
        break;

    case ECU_STATE_RUNNING:
    case ECU_STATE_GOVERNOR_ACTIVE:
        GPIOC->BRR = GPIO_BRR_BR13;
        break;

    case ECU_STATE_FAULT_SAFE_STOP:
        if (led_cnt % 10 == 0) GPIOC->ODR ^= GPIO_ODR_ODR13;
        break;

    default:
        GPIOC->BSRR = GPIO_BSRR_BS13;
        break;
    }
}

void HMI_SyncRegisters(void)
{
    g_holding_regs[HMI_REG_STATE]             = (uint16_t)g_ecu.system_state;
    g_holding_regs[HMI_REG_FAULT_CODE]        = (uint16_t)(g_ecu.fault_flags & 0xFFFFu);
    g_holding_regs[HMI_REG_RPM]               = g_ecu.engine_rpm;
    g_holding_regs[HMI_REG_TPS_RAW]           = g_ecu.governor.actual_tps1_raw;
    g_holding_regs[HMI_REG_THROTTLE_FB]       = g_ecu.governor.actual_tps1_raw;
    g_holding_regs[HMI_REG_IGN_ADVANCE_X1000] = (uint16_t)(g_ecu.ignition.advance_deg_x10 * 100u);
    g_holding_regs[HMI_REG_DWELL_US]          = g_ecu.ignition.dwell_time_us;
    g_holding_regs[HMI_REG_CRANK_SYNC]        = (uint16_t)g_ecu.crank.sync_state;
    g_holding_regs[HMI_REG_UPTIME_SEC]        = (uint16_t)(g_ecu.uptime_ms / 1000u);
}

uint16_t HMI_GetRegister(uint16_t index)
{
    if (index < MODBUS_HOLDING_REGS_COUNT) {
        return g_holding_regs[index];
    }
    return 0u;
}

void HMI_SetRegister(uint16_t index, uint16_t value)
{
    if (index < MODBUS_HOLDING_REGS_COUNT) {
        g_holding_regs[index] = value;
    }
}

const char *HMI_GetStateName(ECU_State_t state)
{
    switch (state) {
        case ECU_STATE_INIT:            return "INIT";
        case ECU_STATE_STANDBY:         return "STANDBY";
        case ECU_STATE_CRANKING:        return "CRANKING";
        case ECU_STATE_RUNNING:         return "RUNNING";
        case ECU_STATE_GOVERNOR_ACTIVE: return "GOVERNOR_ACTIVE";
        case ECU_STATE_FAULT_SAFE_STOP: return "FAULT_SAFE_STOP";
        default:                        return "UNKNOWN";
    }
}

const char *HMI_GetFaultName(ECU_FaultCode_t fault)
{
    switch (fault) {
        case ECU_FAULT_NONE:             return "NO_FAULT";
        case ECU_FAULT_TPS_PLAUSIBILITY: return "TPS_PLAUSIBILITY";
        case ECU_FAULT_OVERSPEED:        return "OVERSPEED";
        case ECU_FAULT_CRANK_LOSS:       return "CRANK_LOSS";
        case ECU_FAULT_BATTERY_LOW:      return "BATTERY_LOW";
        case ECU_FAULT_MAP_SENSOR:       return "MAP_SENSOR";
        default:                         return "UNKNOWN_FAULT";
    }
}

/* ======================= END OF FILE ====================================== */


