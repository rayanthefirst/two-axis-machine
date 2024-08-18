/**
 * @file tam_commands.h
 * @brief Core interface and commands for controlling the two axis machine
 */

#ifndef TAM_COMMANDS_H
#define TAM_COMMANDS_H

#include "tam_context.h"
#include "tam_error.h"

/**
 * @brief Numerical ids for each axis 
 */
typedef enum {
  TAM_AXIS_X = 0,
  TAM_AXIS_Y = 1,
} TAM_Axis;

/**
 * @brief Numerical ID for the direction of movement
 */
typedef enum {
  TAM_AXIS_DIRECTION_POSITIVE = 1,
  TAM_AXIS_DIRECTION_NEGATIVE = -1,
} TAM_AxisDirection;

TAM_Status TAM_AxisTravelSafe(TAM_Axis_Context const * const pContext, int dir);
#define TAM_TravelSafeX(pContext, dir) TAM_AxisTravelSafe(&((pContext)->xAxis), (dir))
#define TAM_TravelSafeY(pContext, dir) TAM_AxisTravelSafe(&((pContext)->yAxis), (dir))
#define TAM_TravelSafe(pContext, dirX, dirY) (TAM_TravelSafeX((pContext), (dirX)) && TAM_TravelSafeY((pContext), (dirY)))


TAM_Status TAM_Move(TAM_Context const * const pContext, float xPos, float yPos);
TAM_Status TAM_AxisMove(TAM_Context const * const pContext, TAM_Axis axis, float pos);
#define TAM_MoveX(pContext, pos) TAM_Move(pContext, pos, 0)
#define TAM_MoveY(pContext, pos) TAM_Move(pContext, 0, pos)

TAM_Status TAM_Run(TAM_Context const * const pContext, float speedX, float speedY);
TAM_Status TAM_AxisMove(TAM_Context const * const pContext, TAM_Axis axis, float speed);
#define TAM_RunX(pContext, speed) TAM_Run(pContext, speed, 0)
#define TAM_RunY(pContext, speed) TAM_Run(pContext, 0, speed)

//void TAM_MovePos(TAM_Context const * const pContext, float xPos, float yPos);
//void TAM_MovePosX(TAM_Context const * const pContext, float pos);
//void TAM_MovePosY(TAM_Context const * const pContext, float pos);

//void TAM_MoveHome(TAM_Context const * const pContext);
//void TAM_AxisMoveHome(TAM_Context const * const pContext);

TAM_Status TAM_SoftStop(TAM_Context const * const pContext);
//void TAM_SoftStopX(TAM_Context const * const pContext);
//void TAM_SoftStopY(TAM_Context const * const pContext);

TAM_Status TAM_HardStop(TAM_Context const * const pContext);
//void TAM_HardStopX(TAM_Context const * const pContext);
//void TAM_HardStopY(TAM_Context const * const pContext);


//void TAM_GetPos(TAM_Context const * const pContext, float * const pXPos, float * const pYPos);
//float TAM_GetPosX(TAM_Context const * const pContext);
//float TAM_GetPosY(TAM_Context const * const pContext);

//void TAM_SetHome(TAM_Context const * const pContext);
//void TAM_SetHomeX(TAM_Context const * const pContext);
//void TAM_SetHomeY(TAM_Context const * const pContext);

TAM_Status TAM_SetAcc(TAM_Context * const pContext, const float accX, const float accY);
#define TAM_SetAccX(pContext, acc) TAM_SetAcc(pContext, acc, pContext->yAxis.kinematicParams.acc)
#define TAM_SetAccY(pContext, acc) TAM_SetAcc(pContext, pContext->xAxis.kinematicParams.acc, acc)

TAM_Status TAM_SetDec(TAM_Context * const pContext, const float decX, const float decY);
#define TAM_SetDecX(pContext, dec) TAM_SetDec(pContext, dec, pContext->yAxis.kinematicParams.dec)
#define TAM_SetDecY(pContext, dec) TAM_SetDec(pContext, pContext->xAxis.kinematicParams.dec, dec)

TAM_Status TAM_SetMaxSpeed(TAM_Context * const pContext, const float maxSpeedX, const float maxSpeedY);
#define TAM_SetMaxSpeedX(pContext, maxSpeed) TAM_SetMaxSpeed(pContext, maxSpeed, pContext->yAxis.kinematicParams.maxSpeed)
#define TAM_SetMaxSpeedY(pContext, maxSpeed) TAM_SetMaxSpeed(pContext, pContext->xAxis.kinematicParams.maxSpeed, maxSpeed)

TAM_Status TAM_SetMinSpeed(TAM_Context * const pContext, const float minSpeedX, const float minSpeedY);
#define TAM_SetMinSpeedX(pContext, minSpeed) TAM_SetMinSpeed(pContext, minSpeed, pContext->yAxis.kinematicParams.minSpeed)
#define TAM_SetMinSpeedY(pContext, minSpeed) TAM_SetMinSpeed(pContext, pContext->xAxis.kinematicParams.minSpeed, minSpeed)

TAM_Status TAM_SetStepSel(TAM_Context * const pContext, const eMotorStepMode_t stepSelM0, const eMotorStepMode_t stepSelM1);
#define TAM_SetStepSelM0(pContext, stepSel) TAM_SetStepSel(pContext, stepSel, pContext->m1Context.params.step_sel)
#define TAM_SetStepSelM1(pContext, stepSel) TAM_SetStepSel(pContext, pContext->m0Context.params.step_sel, stepSel)

TAM_Status TAM_SetAlarmEn(TAM_Context * const pContext, const sL6470_AlarmEnRegister_t alarmEn);

TAM_Status TAM_SetKinematicParams(TAM_Context * const pContext, TAM_AxisKinematicParameters const * const paramsX, TAM_AxisKinematicParameters const * const paramsY);
#define TAM_SetAxisParamsX(pContext, params) TAM_SetAxisParams(pContext, params, &(pContext->yAxis.kinematicParams))
#define TAM_SetAxisParamsY(pContext, params) TAM_SetAxisParams(pContext, &(pContext->xAxis.kinematicParams), params)

TAM_Status TAM_SetDefaultParams(TAM_Context * const pContext);

TAM_Status TAM_GetMotorStatus(TAM_Context const * const pContext, sL6470_StatusRegister_t * const pStatusM0, sL6470_StatusRegister_t * const pStatusM1);

#endif