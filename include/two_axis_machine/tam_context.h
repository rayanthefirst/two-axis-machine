/**
 * @file tam_context.h
 * @brief Two axis machine context struct definition
 */

#ifndef TAM_CONTEXT_H
#define TAM_CONTEXT_H

#include "tam_config.h"
#include "tam_error.h"
#include "xnucleoihm02a1/xnucleoihm02a1.h"

/**
 * @brief The context parameters for the motor
 */
typedef struct {
    TAM_Motor_Config* pConfig;                      //!< The motor config
    uint8_t driverId;                               //!< The driver ID obtained from position (usually the same)
    StepperMotorDriverHandle_t* pDriverHandle;      //!< The driver handle for invoking commands
    MotorParameterData_t params;                    //!< The motor parameter data

    float fullStepsPerRad;                          //!< The number of full steps per radian
    float microStepsPerRad;                         //!< The number of microsteps per radian

} TAM_Motor_Context;

/**
 * @brief The context parameters for a platform axis
 */
typedef struct {
    TAM_Axis_Config* pConfig;                       //!< The axis config
    TAM_AxisKinematicParameters kinematicParams;    //!< The current axis kinematic parameters
} TAM_Axis_Context;

/**
 * @brief The main context of the two axis machine
 */
typedef struct {
    TAM_Config * pConfig;                           //!< The two axis machine config

    TAM_Axis_Context xAxis;                         //!< The x-axis axis context
    TAM_Axis_Context yAxis;                         //!< The y-axis axis context

    uint8_t boardId;                                //!< The board id obtained from the board position
    StepperMotorBoardHandle_t* pBoardHandle;        //!< The board handle for invoking commands

    TAM_Motor_Context m0Context;                    //!< The motor 0 context
    TAM_Motor_Context m1Context;                    //!< The motor 1 context

    // Useful constants to keep
    float motorPulleyRadius;                        //!< The motor pulley radius
    float platformPulleyRadius;                     //!< The platform pulley radius
    float yAxisScrewLeadRadians;                    //!< The y-axis lead screw lead in radians

    // m0Angle = x / motorPulleyRadius
    // m1Angle = (platformPulleyRadius * y - yAxisScrewLeadRadians * x) / (yAxisScrewLeadRadians * motorPulleyRadius)
    
    float xAxisToM0AngleKinFactor;                  //!< The kinematic factor converting motion in x-axis to radians for motor 0.
    float xAxisToM1AngleKinFactor;                  //!< The kinematic factor converting motion in x-axis to radians for motor 1.
    float yAxisToM1AngleKinFactor;                  //!< The kinematic factor converting motion in y-axis to radians for motor 1.

    float xAxisToM0FullStepsKinFactor;              //!< The kinematic factor converting motion in x-axis to full steps for motor 0.          
    float xAxisToM1FullStepsKinFactor;              //!< The kinematic factor converting motion in x-axis to full steps for motor 1.          
    float yAxisToM1FullStepsKinFactor;              //!< The kinematic factor converting motion in y-axis to full steps for motor 1.

    // Changing constants
    float xAxisToM0MicroStepsKinFactor;             //!< The kinematic factor converting motion in x-axis to micro steps for motor 0.
    float xAxisToM1MicroStepsKinFactor;             //!< The kinematic factor converting motion in x-axis to micro steps for motor 1.
    float yAxisToM1MicroStepsKinFactor;             //!< The kinematic factor converting motion in y-axis to micro steps for motor 1.
} TAM_Context;

TAM_Status TAM_Create_Context(TAM_Config const * const pConfig, TAM_Context * const pContext);
TAM_Status TAM_Delete_Context(TAM_Context * const pContext);

#endif