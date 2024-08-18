/**
 * @file tam_motor_commands.h
 * @brief Core interface for running both motors in unison
 */

#ifndef TAM_MOTOR_COMMANDS_H
#define TAM_MOTOR_COMMANDS_H

#include "tam_context.h"
#include "tam_error.h"

TAM_Status TAM_Motor_GoTo(TAM_Context const * const pContext, float posM0, float posM1);
TAM_Status TAM_Motor_Move(TAM_Context const * const pContext, float posM0, float posM1);
TAM_Status TAM_Motor_Run(TAM_Context const * const pContext, float speedM0, float speedM1);
TAM_Status TAM_Motor_SetHome(TAM_Context const * const pContext);
TAM_Status TAM_Motor_GoHome(TAM_Context const * const pContext);
TAM_Status TAM_Motor_SoftStop(TAM_Context const * const pContext);
TAM_Status TAM_Motor_HardStop(TAM_Context const * const pContext);

TAM_Status TAM_Motor_GetParam(TAM_Context const * const pContext, const eL6470_RegId_t regId, uint32_t * const paramM0, uint32_t * const paramM1);
TAM_Status TAM_Motor_GetStatus(TAM_Context const * const pContext, sL6470_StatusRegister_t * const pStatusM0, sL6470_StatusRegister_t * const pStatusM1);

TAM_Status TAM_Motor_SetParam(TAM_Context * const pContext, const eL6470_RegId_t regId, const uint32_t paramM0, const uint32_t paramM1);
TAM_Status TAM_Motor_SetAcc(TAM_Context * const pContext, const float accM0, const float accM1);
TAM_Status TAM_Motor_SetDec(TAM_Context * const pContext, const float decM0, const float decM1);
TAM_Status TAM_Motor_SetMaxSpeed(TAM_Context * const pContext, const float maxSpeedM0, const float maxSpeedM1);
TAM_Status TAM_Motor_SetMinSpeed(TAM_Context * const pContext, const float minSpeedM0, const float minSpeedM1);
TAM_Status TAM_Motor_SetStepSel(TAM_Context * const pContext, const eMotorStepMode_t stepSelM0, const eMotorStepMode_t stepSelM1);
TAM_Status TAM_Motor_SetAlarmEn(TAM_Context * const pContext, const sL6470_AlarmEnRegister_t alarmEnM0, const sL6470_AlarmEnRegister_t alarmEnM1);

TAM_Status TAM_Motor_SetParams(TAM_Context * const pContext, MotorParameterData_t const * const pParamsM0, MotorParameterData_t const * const pParamsM1);
TAM_Status TAM_Motor_SetDefaultParams(TAM_Context * const pContext);

#endif