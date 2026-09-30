/*
 * ecu.h
 *
 *  Created on: ۱۴ سپتامبر ۲۰۲۶
 *      Author: rm
 */

/**
 * @file    ecu_types.h
 * @brief   Data types, enumerations, and system runtime structures for Mini-ECU.
 * @target  STM32F103 (Register-Level / Bare-Metal)
 */

#ifndef ECU_TYPES_H_
#define ECU_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 1. ENUMERATIONS (سیستم وضعیت و کدهای خطا)
 * ============================================================================ */

/**
 * @brief وضعیت‌های عملیاتی کلان ECU
 */
typedef enum {
    ECU_STATE_INIT = 0,        /* مقداردهی رجیسترها و کلاک */
    ECU_STATE_STANDBY,         /* آماده‌به‌کار، موتور خاموش */
    ECU_STATE_CRANKING,        /* در حال استارت‌خوردن (دور زیر 450 RPM) */
    ECU_STATE_RUNNING,         /* موتور روشن و پایدار */
    ECU_STATE_GOVERNOR_ACTIVE, /* کنترل فعال دریچه گاز جهت تثبیت دور ژنراتور */
    ECU_STATE_FAULT_SAFE_STOP  /* بروز خطای بحرانی و قطع فوری خروجی‌ها */
} ECU_State_t;

/**
 * @brief وضعیت همگام‌سازی موقعیت میل‌لنگ (Crank 60-2 Sync)
 */
typedef enum {
    CRANK_SYNC_LOST = 0,       /* عدم تشخیص موقعیت یا بروز خطا در فاصله دندانه‌ها */
    CRANK_SYNC_SEARCHING,      /* در حال شمارش دندانه‌ها و پیدا کردن گپ ۲ دندانه */
    CRANK_SYNC_GAP_FOUND,      /* گپ ۲ دندانه خالی کشف شد */
    CRANK_SYNC_LOCKED          /* همگام‌سازی قطعی؛ موقعیت دندانه ۱ تا ۵۸ معتبر است */
} CrankSyncState_t;

/**
 * @brief وضعیت سیستم ایمنی پدال/دریچه گاز (Dual TPS Plausibility)
 */
typedef enum {
    TPS_STATUS_OK = 0,         /* سنسورها سالم و تطابق کامل دارند */
    TPS_STATUS_ERR_SHORT_GND,  /* اتصال کوتاه به زمین */
    TPS_STATUS_ERR_SHORT_VCC,  /* اتصال کوتاه به تغذیه */
    TPS_STATUS_ERR_MISMATCH    /* عدم انطباق سنسور ۱ و ۲ (اختلاف جمع از ۴۰۹۵) */
} TPS_Status_t;

/**
 * @brief وضعیت سلکتور کویل‌های جرقه (Waste-Spark A/B)
 */
typedef enum {
    COIL_NONE = 0,
    COIL_A,                    /* خروجی PA6 (سیلندرهای 1 و 4) */
    COIL_B                     /* خروجی PA7 (سیلندرهای 2 و 3) */
} CoilSelect_t;

/* --- Fault Flags Bitmask (خطاهای بحرانی سیستم) --- */
typedef enum {
    ECU_FAULT_NONE             = 0,
    ECU_FAULT_TPS_PLAUSIBILITY = (1UL << 0),  /* بیت 0: عدم تطابق سنسورهای دوقلوی دریچه گاز */
    ECU_FAULT_OVERSPEED        = (1UL << 1),  /* بیت 1: رد شدن از دور مجاز موتور */
    ECU_FAULT_CRANK_LOSS       = (1UL << 2),  /* بیت 2: قطع سیگنال دور موتور */
    ECU_FAULT_BATTERY_LOW      = (1UL << 3),  /* بیت 3: افت ولتاژ تغذیه */
    ECU_FAULT_MAP_SENSOR       = (1UL << 4)   /* بیت 4: خطای سنسور فشار منیفولد */
} ECU_FaultCode_t;

/* ============================================================================
 * 2. STRUCTURES (ساختارهای داده موتور و زیرسیستم‌ها)
 * ============================================================================ */

/**
 * @brief ساختار ردگیری دندانه‌ها و زمان‌بندی زاویه‌ای میل‌لنگ
 */
typedef struct {
    volatile uint32_t last_tooth_time_us;   /* زمان لبه دندانه قبلی (میکروثانیه) */
    volatile uint32_t tooth_period_us;      /* مدت‌زمان دندانه جاری (فاصله دو لبه) */
    volatile uint32_t avg_tooth_period_us;  /* میانگین فیلترشده پریود دندانه‌ها */
    volatile uint8_t  current_tooth;        /* شماره دندانه فعلی (1 تا 58) */
    volatile CrankSyncState_t sync_state;   /* وضعیت سنکرون میل‌لنگ */
    volatile uint32_t tooth_counter_raw;    /* شمارنده خام پالس‌ها جهت دیباگ */
} Crank_Telemetry_t;

/**
 * @brief ساختار زمان‌بندی جرقه (Dwell و Advance)
 */
typedef struct {
    volatile uint16_t advance_deg_x10;      /* زاویه پیش‌افروزش (مثلاً 100 = 10.0 درجه) */
    volatile uint16_t dwell_time_us;        /* زمان شارژ کویل بر حسب میکروثانیه (Dwell) */
    volatile uint8_t  fire_tooth_coil_a;    /* شماره دندانه تریگر شروع شارژ کویل A */
    volatile uint8_t  fire_tooth_coil_b;    /* شماره دندانه تریگر شروع شارژ کویل B */
    volatile CoilSelect_t next_coil;        /* کویل نوبت بعد برای شارژ/جرقه */
    volatile uint32_t spark_fire_time_us;
    volatile uint8_t  spark_armed;
    volatile bool     dwell_active;         /* وضعیت روشن بودن کویل */
} Ignition_Control_t;

/**
 * @brief وضعیت TPS و فرمان عملگر دریچه گاز برقی
 */
typedef struct {
    volatile uint16_t target_tps_raw;      /* هدف TPS1 پس از کالیبراسیون، 0..4095 */
    volatile uint16_t actual_tps1_raw;     /* مقدار خام TPS1 از PA1 */
    volatile uint16_t actual_tps2_raw;     /* مقدار خام TPS2 از PA5 */
    volatile TPS_Status_t tps_health;      /* وضعیت سلامت و تطابق TPS */
    volatile int16_t  pwm_duty;            /* -1000..+1000؛ علامت=جهت */
    volatile bool     hbridge_enabled;     /* فعال‌بودن Enable درایور IBT-2، PB5 */
    volatile uint16_t target_rpm;          /* هدف دور گاورنر */
} Governor_Actuator_t;

/**
 * @brief ساختار جامع و یکپارچه وضعیت کل ECU (Runtime Struct)
 */
typedef struct {
    volatile ECU_State_t system_state;     /* وضعیت کلی کنترلر */
    volatile uint32_t uptime_ms;           /* زمان روشن بودن به میلی‌ثانیه (Tick) */
    volatile uint16_t engine_rpm;          /* دور موتور لحظه‌ای محاسبه‌شده */
    Crank_Telemetry_t crank;               /* پارامترهای سنسور میل‌لنگ */
    Ignition_Control_t ignition;           /* پارامترهای سیستم جرقه */
    Governor_Actuator_t governor;          /* پارامترهای گاورنر و دریچه گاز */
    volatile uint32_t fault_flags;         /* بیت‌های ثبات خطا (Bitmask) */
} ECU_Runtime_t;

#endif /* ECU_TYPES_H_ */

