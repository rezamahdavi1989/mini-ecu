#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include "ecu_types.h"
#include "config.h"  

/*
 * =====================================================================================
 *  Mini-ECU  |  HMI / Telemetry Register Map  (Modbus-Ready Flat Address Space)
 * =====================================================================================
 */

typedef enum {
    /* ---- Status & Telemetry (Read-Only) ---- */
    HMI_REG_STATE               = 0,   /* RO | وضعیت فعلی ماشین وضعیت (ECU_State_t)              */
    HMI_REG_FAULT_CODE          = 1,   /* RO | کد خطای فعال فعلی (ECU_FaultCode_t)               */
    HMI_REG_RPM                 = 2,   /* RO | دور واقعی موتور (Engine RPM)                      */
    HMI_REG_TARGET_RPM          = 3,   /* RO | دور هدف فعال گاورنر                                */
    HMI_REG_BATTERY_MV          = 4,   /* RO | ولتاژ باتری به میلی‌ولت                            */
    HMI_REG_TPS_RAW             = 5,   /* RO | خواندن خام سنسور PA1 (TPS1)                       */
    
    /* ---- Configuration & Calibration (Read/Write) ---- */
    HMI_REG_TPS_MIN             = 6,   /* RW | حداقل کالیبراسیون TPS                              */
    HMI_REG_TPS_MAX             = 7,   /* RW | حداکثر کالیبراسیون TPS                              */
    
    /* ---- Actuator Status & Ignition (Read/Write & Read-Only) ---- */
    HMI_REG_THROTTLE_CMD        = 8,   /* RO | فرمان نهایی بازشدن دریچه گاز                      */
    HMI_REG_THROTTLE_FB         = 9,   /* RO | فیدبک واقعی موقعیت دریچه (actual_tps1_raw)        */
    HMI_REG_IGN_ADVANCE_X1000   = 10,  /* RW | پیش‌افت جرقه (Advance Angle) × 1000 درجه          */
    HMI_REG_DWELL_US            = 11,  /* RW | زمان شارژ کویل (Dwell Time) به میکروثانیه          */
    
    /* ---- Governor PID Parameters (Read/Write) ---- */
    HMI_REG_KP_X1000            = 12,  /* RW | ضریب تناسبی گاورنر (Kp) × 1000                    */
    HMI_REG_KI_X1000            = 13,  /* RW | ضریب انتگرالی گاورنر (Ki) × 1000                  */
    HMI_REG_KD_X1000            = 14,  /* RW | ضریب مشتقی گاورنر (Kd) × 1000                     */
    HMI_REG_OVERSPEED_TRIP_RPM  = 15,  /* RW | آستانه خاموشی اضطراری دور اضافه                    */
    
    /* ---- Engine Sync & Control Commands (Read/Write & Read-Only) ---- */
    HMI_REG_CRANK_SYNC          = 16,  /* RO | وضعیت همگام‌سازی میل‌لنگ (CrankSyncState_t)         */
    HMI_REG_ENGINE_ENABLE       = 17,  /* RW | فرمان کلی روشن/خاموش موتور                         */
    HMI_REG_IGNITION_ENABLE     = 18,  /* RW | فعال‌سازی خروجی سیستم جرقه                        */
    HMI_REG_GOVERNOR_ENABLE     = 19,  /* RW | فعال‌سازی کنترلر دریچه گاز و گاورنر                */
    HMI_REG_FAULT_LATCH         = 20,  /* RW | ریست وضعیت خطا (نوشتن 0 = ریست فالت‌ها)            */
    HMI_REG_STARTER_REQUEST     = 21,  /* RW | فرمان درگیر شدن رله استارت                         */
    HMI_REG_FUEL_ENABLE         = 22,  /* RW | فرمان شیر برقی یا رله سوخت                        */
    HMI_REG_SAFE_TO_RUN         = 23,  /* RO | وضعیت ایمنی (1 = مجاز به کار، 0 = خطا)             */
    HMI_REG_UPTIME_SEC          = 24,  /* RO | مدت زمان روشن بودن سیستم به ثانیه                  */
    
    /* ---- Reserved Registers ---- */
    HMI_REG_RESERVED_00         = 25,  
    HMI_REG_RESERVED_01         = 26,  
    HMI_REG_RESERVED_02         = 27,  
    HMI_REG_RESERVED_03         = 28,  
    HMI_REG_RESERVED_04         = 29,  
    HMI_REG_RESERVED_05         = 30,  
    HMI_REG_RESERVED_06         = 31,  
    
    /* ---- Sentinel ---- */
    HMI_REG_COUNT               = 32   /* تعداد کل رجیسترها                                      */
} HMI_Register_t;

/* ---------------------------------------------------------------------------------
 *  API عمومی ماژول HMI
 * --------------------------------------------------------------------------------- */
void        HMI_Init(void);
void        HMI_UpdateLED(ECU_State_t state);
void        HMI_SyncRegisters(void);

uint16_t    HMI_GetRegister(uint16_t index);
void        HMI_SetRegister(uint16_t index, uint16_t value);

const char *HMI_GetStateName(ECU_State_t state);
const char *HMI_GetFaultName(ECU_FaultCode_t fault);

#endif /* HMI_H */
