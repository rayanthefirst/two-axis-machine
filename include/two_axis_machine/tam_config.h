/**
 * @file tam_config.h
 * @brief Defines the interface and struct definitions for configuration of the two axis machine
 */

#ifndef TAM_CONFIG_H
#define TAM_CONFIG_H

#include "gpio_utils.h"
#include "xnucleoihm02a1/microstepping_motor.h"
#include "stdbool.h"

/**
 * @brief Contains the config/initialization parameters for a limit switch on the two axis machine
 */
typedef struct {
    GPIO_Pin pin;                           //!< The GPIO pin the switch is connected to
    GPIO_PullMode pull;                     //!< The GPIO pull mode to configure
    GPIO_PinState activeState;              //!< The state at which the limit switch is actually pressed
    EXTI_Config extiConfig;                 //!< The configuration for the external interrupts like priority 
    uint32_t debounceStableTimeTol_ms;      //!< The debounce time tolerance before the switch can be considered stable
    uint32_t debounceUnstableTimeout_ms;    //!< The timeout for how long a swithc can be in the "unstable" state
    bool setHome;                           //!< Whether this limit switch is the "home" for the axis
} TAM_Axis_Lim_Swt_Config;

/**
 * @brief Contains the kinematic settings for the axis describing movement
 */
typedef struct {
    float acc;                              //!< The axis acceleration in distance / s^2
    float dec;                              //!< The axis deceleration in distance / s^2
    float maxSpeed;                         //!< The axis maximum speed in distance / s
    float minSpeed;                         //!< The axis minimum speed in distance / s
} TAM_AxisKinematicParameters;

/**
 * @brief Contains the config parameters for an axis
 */
typedef struct {
    TAM_Axis_Lim_Swt_Config minLimSwtConfig;                //!< The limit switch config for the switch at the minimum position
    TAM_Axis_Lim_Swt_Config maxLimSwtConfig;                //!< The limit switch config for the switch at the maximum position
    TAM_AxisKinematicParameters initialKinematicParams;     //!< The initial/default kinematic parameters configured for the axis
} TAM_Axis_Config;

/**
 * @brief The config parameters for a motor
 */
typedef struct {
    uint8_t driverPosition;                                 //!< L6470 position
    float fullStepsPerRev;                                  //!< The number of steps per revolution, usually 200
    eMotorStepMode_t stepSel;                               //!< The step selection for the motor
} TAM_Motor_Config;

/**
 * @brief The overall configuration for the two axis machine 
 */
typedef struct {
    uint8_t boardPosition;                                  //!< The IHM02A1 shield position (0 for one shield)
    TAM_Axis_Config xAxis;                                  //!< The x-axis configuration settings
    TAM_Axis_Config yAxis;                                  //!< The y-axis configuration settings

    TAM_Motor_Config m0Config;                              //!< Motor 0 configuration settings
    TAM_Motor_Config m1Config;                              //!< Motor 1 configuration settings
    sL6470_AlarmEnRegister_t alarmEn;                       //!< Alarm enable settings for the motors

    float beltPitch;                                        //!< The pulley belt pitch (same units as everything else)

    unsigned int motorPulleyTeeth;                          //!< The number of teeth on the main motor pulleys
    unsigned int platformPulleyTeeth;                       //!< The number of teeth on the platform pulley
    float yAxisScrewLead;                                   //!< The lead of the lead screw on the platform y-axis mechanism

} TAM_Config;

#endif
