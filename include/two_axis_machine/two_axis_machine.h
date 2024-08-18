#ifndef TAM_HARDWARE_CONFIG_H
#define TAM_HARDWARE_CONFIG_H

#include "gpio_utils.h"
#include "two_axis_machine/tam_config.h"
#include "two_axis_machine/tam_context.h"
#include "two_axis_machine/tam_commands.h"

#define TAM_LIM_SWT_PULLMODE                        GPIO_PULLMODE_PULLUP
#define TAM_LIM_SWT_ACTIVE_STATE                    GPIO_PIN_SET

#define TAM_LIM_SWT_EXTI_PREEMPT_PRIORITY           PRIORITY_LEVEL_HIGH_SAFETY
#define TAM_LIM_SWT_EXTI_SUB_PRIORITY               0x00

#define TAM_LIM_SWT_DEBOUNCE_STABLE_TIME_TOL_MS     20
#define TAM_LIM_SWT_DEBOUNCE_UNSTABLE_TIMEOUT_MS    50

static const TAM_Config TAM_CONFIG = {
    .boardPosition = 0,
    .xAxis = {
        .minLimSwtConfig = {
            .pin = {
                .port = GPIOB,
                .pin = GPIO_PIN_8
            },
            .pull = TAM_LIM_SWT_PULLMODE,
            .activeState = TAM_LIM_SWT_ACTIVE_STATE,
            .extiConfig = {
              .preemptPriority = TAM_LIM_SWT_EXTI_PREEMPT_PRIORITY,
              .subPriority = TAM_LIM_SWT_EXTI_SUB_PRIORITY,
            },
            .debounceStableTimeTol_ms = TAM_LIM_SWT_DEBOUNCE_STABLE_TIME_TOL_MS,
            .debounceUnstableTimeout_ms = TAM_LIM_SWT_DEBOUNCE_UNSTABLE_TIMEOUT_MS,
            .setHome = true,
        },
        .maxLimSwtConfig = {
            .pin = {
                .port = GPIOB,
                .pin = GPIO_PIN_9
            },
            .pull = TAM_LIM_SWT_PULLMODE,
            .activeState = TAM_LIM_SWT_ACTIVE_STATE,
            .extiConfig = {
              .preemptPriority = TAM_LIM_SWT_EXTI_PREEMPT_PRIORITY,
              .subPriority = TAM_LIM_SWT_EXTI_SUB_PRIORITY,
            },
            .debounceStableTimeTol_ms = TAM_LIM_SWT_DEBOUNCE_STABLE_TIME_TOL_MS,
            .debounceUnstableTimeout_ms = TAM_LIM_SWT_DEBOUNCE_UNSTABLE_TIMEOUT_MS,
            .setHome = false,
        },
        .initialKinematicParams = {
            .acc = 5,
            .dec = 5,
            .maxSpeed = 5,
            .minSpeed = 0.1,
        },
    },
    .yAxis = {
        .minLimSwtConfig = {
            .pin = {
                .port = GPIOB,
                .pin = GPIO_PIN_10
            },
            .pull = TAM_LIM_SWT_PULLMODE,
            .activeState = TAM_LIM_SWT_ACTIVE_STATE,
            .extiConfig = {
              .preemptPriority = TAM_LIM_SWT_EXTI_PREEMPT_PRIORITY,
              .subPriority = TAM_LIM_SWT_EXTI_SUB_PRIORITY,
            },
            .debounceStableTimeTol_ms = TAM_LIM_SWT_DEBOUNCE_STABLE_TIME_TOL_MS,
            .debounceUnstableTimeout_ms = TAM_LIM_SWT_DEBOUNCE_UNSTABLE_TIMEOUT_MS,
            .setHome = true,
        },
        .maxLimSwtConfig = {
            .pin = {
                .port = GPIOB,
                .pin = GPIO_PIN_4
            },
            .pull = TAM_LIM_SWT_PULLMODE,
            .activeState = TAM_LIM_SWT_ACTIVE_STATE,
            .extiConfig = {
              .preemptPriority = TAM_LIM_SWT_EXTI_PREEMPT_PRIORITY,
              .subPriority = TAM_LIM_SWT_EXTI_SUB_PRIORITY,
            },
            .debounceStableTimeTol_ms = TAM_LIM_SWT_DEBOUNCE_STABLE_TIME_TOL_MS,
            .debounceUnstableTimeout_ms = TAM_LIM_SWT_DEBOUNCE_UNSTABLE_TIMEOUT_MS,
            .setHome = false,
        },
        .initialKinematicParams = {
            .acc = 5,
            .dec = 5,
            .maxSpeed = 5,
            .minSpeed = 0.1,
        },
    },
    .m0Config = {
        .driverPosition = 0,
        .fullStepsPerRev = 200,
        .stepSel = HALF_STEP,
    },
    .m1Config = {
        .driverPosition = 1,
        .fullStepsPerRev = 200,
        .stepSel = HALF_STEP,
    },
    .alarmEn = {
        .STEP_LOSS_A_EN = 1,
        .STEP_LOSS_B_EN = 1,
        .TH_WRN_EN = 1,
        .TH_SD_EN = 1,
        .OCD_EN = 1,
        .SW_EVN_EN = 0,
        .WRONG_NOTPERF_CMD_EN = 1,
        .UVLO_EN = 1
    },

    .beltPitch = 0.508, // cm
    .motorPulleyTeeth = 24,
    .platformPulleyTeeth = 32,
    .yAxisScrewLead = 3 // cm
};

extern TAM_Context tamContext;

TAM_Status TAM_Init();
TAM_Status TAM_DeInit();


#endif