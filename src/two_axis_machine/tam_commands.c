#include "two_axis_machine/tam_commands.h"
#include "string.h"
#include "two_axis_machine/tam_motor_commands.h"
#include "xnucleoihm02a1/xnucleoihm02a1.h"
#include "trace.h"

#define TAM_ApplyKinematicFactor_M0(pContext, xVal, yVal) ((xVal) * (pContext)->xAxisToM0FullStepsKinFactor)
#define TAM_ApplyKinematicFactor_M1(pContext, xVal, yVal) ((yVal) * (pContext)->yAxisToM1FullStepsKinFactor - (xVal) * (pContext)->xAxisToM1FullStepsKinFactor)

#define TAM_ApplyKinematicFactorMS_M0(pContext, xVal, yVal) ((xVal) * (pContext)->xAxisToM0MicroStepsKinFactor)
#define TAM_ApplyKinematicFactorMS_M1(pContext, xVal, yVal) ((yVal) * (pContext)->yAxisToM1MicroStepsKinFactor - (xVal) * (pContext)->xAxisToM1MicroStepsKinFactor)

/**
 * @brief Determines if the movement in the direction specified is blocked for the axis
 * 
 * @param pContext      The two axis machine context struct
 * @param dir           The direction of motion to check
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_AxisTravelSafe(TAM_Axis_Context const * const pContext, int dir)
{
  if (dir == 0)
  {
    return TAM_OK;
  }
  
  TAM_Axis_Lim_Swt_Config const * const blockingSwt = dir > 0 ? 
    &(pContext->pConfig->maxLimSwtConfig) : 
    &(pContext->pConfig->minLimSwtConfig);


  if (HAL_GPIO_ReadPin(blockingSwt->pin.port, blockingSwt->pin.pin) == blockingSwt->activeState)
  {
    return TAM_TRAVEL_BLOCKED;
  }

  return TAM_OK;
}

int signof(const float value)
{
  if (value > 0)
  {
    return 1;
  }

  if (value < 0)
  {
    return -1;
  }

  return 0;
}

#define TAM_AssertTravelSafe(pContext, xValue, yValue)                              \
{                                                                                   \
  const int xDir = signof(xValue);                                                  \
  TAM_CHECK_STATUS_LOG_RETURN(                                                      \
    TAM_TravelSafeX(pContext, (xDir)),                                              \
    "Travel in x-axis (%d) blocked by limit switch, cannot execute command",        \
    (xDir)                                                                          \
  );                                                                                \
                                                                                    \
  const int yDir = signof(yValue);                                                  \
  TAM_CHECK_STATUS_LOG_RETURN(                                                      \
    TAM_TravelSafeY(pContext, yDir),                                                \
    "Travel in y-axis (%d) blocked by limit switch, cannot execute command",        \
    (yDir)                                                                          \
  );                                                                                \
}

/**
 * @brief Moves the platform the specific units of distance in each direction
 * 
 * @param pContext      The two axis machine context struct
 * @param posX          The displacement in the x-direction in distance units (whatever used for the config).
 * @param posY          The displacement in the y-direction in distance units (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_Move(TAM_Context const * const pContext, float posX, float posY)
{
  LOG_DEBUG("Preparing to move the platform %f units in x-direction and %f units in y-direction", posX, posY);
  TAM_AssertTravelSafe(pContext, posX, posY);

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_Move(
      pContext, 
      TAM_ApplyKinematicFactorMS_M0(pContext, posX, posY), 
      TAM_ApplyKinematicFactorMS_M1(pContext, posX, posY)
    ),
    "Error performing 'Move' command for motors"
  );

  LOG_INFO("Moving the platform %f units in x-direction and %f units in y-direction", posX, posY);
  
  return TAM_OK;
}

/**
 * @brief Move the the specific axis the specific units while keeping the other axis stationary
 * 
 * @param pContext      The two axis machine context struct
 * @param axis          The axis to move
 * @param pos           The displacement distance in distance units (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_AxisMove(TAM_Context const * const pContext, TAM_Axis axis, float pos)
{
  if (axis == TAM_AXIS_X)
  {
    return TAM_MoveX(pContext, pos);
  }
  else if (axis == TAM_AXIS_Y)
  {
    return TAM_MoveY(pContext, pos);
  }
  else
  {
    ASSERT_CHECK_LOG_RETURN(false, TAM_INVALID_PARAM, "Unknown Axis %d", axis);
  }
}

/**
 * @brief Run the platform continously at the given velocities for each axis
 * 
 * @param pContext      The two axis machine context struct
 * @param speedX        The velocity in the x-axis in distance units/s (whatever used for the config).
 * @param speedY        The velocity in the y-axis in distance units/s (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_Run(TAM_Context const * const pContext, float speedX, float speedY)
{
  LOG_DEBUG("Preparing to run the platform at %f units/s in x-direction and %f units/s in y-direction", speedX, speedY);
  TAM_AssertTravelSafe(pContext, speedX, speedY);

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_Run(
      pContext, 
      TAM_ApplyKinematicFactor_M0(pContext, speedX, speedY), 
      TAM_ApplyKinematicFactor_M1(pContext, speedX, speedY)
    ),
    "Error performing 'Move' command for motors"
  );

  LOG_INFO("Running platform at %f units/s in x-direction and %f units/s in y-direction", speedX, speedY);

  return TAM_OK;
}

/**
 * @brief Run the platform continously at the given velocity for the given axis
 * 
 * @param pContext      The two axis machine context struct
 * @param axis          The axis to move
 * @param speed         The velocity to move the axis at in distance units/s (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_AxisRun(TAM_Context const * const pContext, TAM_Axis axis, float speed)
{
  if (axis == TAM_AXIS_X)
  {
    return TAM_RunX(pContext, speed);
  }
  else if (axis == TAM_AXIS_Y)
  {
    return TAM_RunY(pContext, speed);
  }
  else
  {
    ASSERT_CHECK_LOG_RETURN(false, TAM_INVALID_PARAM, "Unknown Axis %d", axis);
  }
}

/**
 * @brief Soft stop the platform
 * 
 * @param pContext      The two axis machine context struct
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SoftStop(TAM_Context const * const pContext)
{
  LOG_DEBUG("Preparing to soft stop all axis");
  TAM_CHECK_STATUS_LOG_RETURN(TAM_Motor_SoftStop(pContext), "Error soft stopping the motors");
  LOG_INFO("All axis soft stopped");
  return TAM_OK;
}

/**
 * @brief Hard stop the platform
 * 
 * @param pContext      The two axis machine context struct
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_HardStop(TAM_Context const * const pContext)
{
  LOG_DEBUG("Preparing to hard stop all axis");
  TAM_CHECK_STATUS_LOG_RETURN(TAM_Motor_HardStop(pContext), "Error hard stopping the motors");
  LOG_INFO("All axis hard stopped");
  return TAM_OK;
}

/**
 * @brief Set the acceleration for the platform for each axis
 * 
 * @param pContext      The two axis machine context struct
 * @param accX          The acceleration in the x-axis in distance units/s^2 (whatever used for the config).
 * @param accY          The acceleration in the y-axis in distance units/s^2 (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetAcc(TAM_Context * const pContext, const float accX, const float accY)
{
  ASSERT_CHECK_LOG_RETURN(accX > 0, TAM_INVALID_PARAM, "Invalid zero or negative acceleration for x-axis: %f", accX);
  ASSERT_CHECK_LOG_RETURN(accY > 0, TAM_INVALID_PARAM, "Invalid zero or negative acceleration for y-axis: %f", accY);

  LOG_DEBUG("Preparing to set the x-axis acceleration to %f", accX);
  LOG_DEBUG("Preparing to set the y-axis acceleration to %f", accY);

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_SetAcc(
      pContext, 
      TAM_ApplyKinematicFactor_M0(pContext, accX, accY), 
      TAM_ApplyKinematicFactor_M1(pContext, accX, accY)
    ),
    "Error setting the acceleration"
  );
  
  pContext->xAxis.kinematicParams.acc = accX;
  pContext->yAxis.kinematicParams.acc = accY;

  LOG_INFO("Set the x-axis acceleration to %f", accX);
  LOG_INFO("Set the y-axis acceleration to %f", accY);

  return TAM_OK;
}

/**
 * @brief Set the deceleration for the platform for each axis
 * 
 * @param pContext      The two axis machine context struct
 * @param decX          The deceleration in the x-axis in distance units/s^2 (whatever used for the config).
 * @param decY          The deceleration in the y-axis in distance units/s^2 (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetDec(TAM_Context * const pContext, const float decX, const float decY)
{
  ASSERT_CHECK_LOG_RETURN(decX > 0, TAM_INVALID_PARAM, "Invalid zero or negative deceleration for x-axis: %f", decX);
  ASSERT_CHECK_LOG_RETURN(decY > 0, TAM_INVALID_PARAM, "Invalid zero or negative deceleration for y-axis: %f", decY);

  LOG_DEBUG("Preparing to set the x-axis deceleration to %f", decX);
  LOG_DEBUG("Preparing to set the y-axis deceleration to %f", decY);

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_SetDec(
      pContext, 
      TAM_ApplyKinematicFactor_M0(pContext, decX, decY), 
      TAM_ApplyKinematicFactor_M1(pContext, decX, decY)
    ),
    "Error setting the deceleration"
  );

  pContext->xAxis.kinematicParams.dec = decX;
  pContext->yAxis.kinematicParams.dec = decY;

  LOG_INFO("Set the x-axis deceleration to %f", decX);
  LOG_INFO("Set the y-axis deceleration to %f", decY);

  return TAM_OK;
}

/**
 * @brief Set the max speed for the platform for each axis
 * 
 * @param pContext      The two axis machine context struct
 * @param maxSpeedX     The max speed in the x-axis in distance units/s (whatever used for the config).
 * @param maxSpeedY     The max speed in the y-axis in distance units/s (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetMaxSpeed(TAM_Context * const pContext, const float maxSpeedX, const float maxSpeedY)
{
  ASSERT_CHECK_LOG_RETURN(maxSpeedX > 0, TAM_INVALID_PARAM, "Invalid zero or negative max speed for x-axis: %f", maxSpeedX);
  ASSERT_CHECK_LOG_RETURN(maxSpeedY > 0, TAM_INVALID_PARAM, "Invalid zero or negative max speed for y-axis: %f", maxSpeedY);

  LOG_DEBUG("Preparing to set the x-axis maximum speed to %f", maxSpeedX);
  LOG_DEBUG("Preparing to set the y-axis maximum speed to %f", maxSpeedY);

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_SetMaxSpeed(
      pContext, 
      TAM_ApplyKinematicFactor_M0(pContext, maxSpeedX, maxSpeedY), 
      TAM_ApplyKinematicFactor_M1(pContext, maxSpeedX, maxSpeedY)
    ),
    "Error setting the maximum speed"
  );

  pContext->xAxis.kinematicParams.maxSpeed = maxSpeedX;
  pContext->yAxis.kinematicParams.maxSpeed = maxSpeedY;

  LOG_INFO("Set the x-axis maximum speed to %f", maxSpeedX);
  LOG_INFO("Set the y-axis maximum speed to %f", maxSpeedY);

  return TAM_OK;
}

/**
 * @brief Set the min speed for the platform for each axis
 * 
 * @param pContext      The two axis machine context struct
 * @param minSpeedX     The min speed in the x-axis in distance units/s (whatever used for the config).
 * @param minSpeedY     The min speed in the y-axis in distance units/s (whatever used for the config).
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetMinSpeed(TAM_Context * const pContext, const float minSpeedX, const float minSpeedY)
{
  ASSERT_CHECK_LOG_RETURN(minSpeedX > 0, TAM_INVALID_PARAM, "Invalid zero or negative min speed for x-axis: %f", minSpeedX);
  ASSERT_CHECK_LOG_RETURN(minSpeedY > 0, TAM_INVALID_PARAM, "Invalid zero or negative min speed for y-axis: %f", minSpeedY);

  LOG_DEBUG("Preparing to set the x-axis minimum speed to %f", minSpeedX);
  LOG_DEBUG("Preparing to set the y-axis minimum speed to %f", minSpeedY);

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_SetMinSpeed(
      pContext, 
      TAM_ApplyKinematicFactor_M0(pContext, minSpeedX, minSpeedY), 
      TAM_ApplyKinematicFactor_M1(pContext, minSpeedX, minSpeedY)
    ),
    "Error setting the minimum speed"
  );

  pContext->xAxis.kinematicParams.minSpeed = minSpeedX;
  pContext->yAxis.kinematicParams.minSpeed = minSpeedY;

  LOG_INFO("Set the x-axis minimum speed to %f", minSpeedX);
  LOG_INFO("Set the y-axis minimum speed to %f", minSpeedY);
  
  return TAM_OK;
}

/**
 * @brief Sets a new set of kinematic parameters for each axis
 * 
 * @param pContext      The two axis machine context struct
 * @param paramsX       The new kinematic parameters for the x-axis
 * @param paramsY       The new kinematic parameters for the y-axis
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetKinematicParams(TAM_Context * const pContext, TAM_AxisKinematicParameters const * const paramsX, TAM_AxisKinematicParameters const * const paramsY)
{
  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetAcc(pContext, paramsX->acc, paramsY->acc),
    "Error setting the acceleration"
  );

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetDec(pContext, paramsX->dec, paramsY->dec),
    "Error setting the deceleration"
  );
  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetMaxSpeed(pContext, paramsX->maxSpeed, paramsY->maxSpeed),
    "Error setting the max speed"
  );
  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetMinSpeed(pContext, paramsX->minSpeed, paramsY->minSpeed),
    "Error setting the min speed"
  );

  return TAM_OK;
}

/**
 * @brief Set the step selection for the individual motors
 * 
 * @param pContext      The two axis machine context struct
 * @param stepSelM0     The step selection for motor 0.
 * @param stepSelM1     The step selection for motor 1.
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetStepSel(TAM_Context * const pContext, const eMotorStepMode_t stepSelM0, const eMotorStepMode_t stepSelM1)
{
  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_SetStepSel(pContext, stepSelM0, stepSelM1), 
    "Error setting motor step selection"
  );

  pContext->m0Context.microStepsPerRad = pContext->m0Context.fullStepsPerRad * (1 << stepSelM0);
  pContext->m1Context.microStepsPerRad = pContext->m1Context.fullStepsPerRad * (1 << stepSelM1);

  pContext->xAxisToM0MicroStepsKinFactor = pContext->xAxisToM0AngleKinFactor * pContext->m0Context.microStepsPerRad;
  pContext->xAxisToM1MicroStepsKinFactor = pContext->xAxisToM1AngleKinFactor * pContext->m1Context.microStepsPerRad;
  pContext->yAxisToM1MicroStepsKinFactor = pContext->yAxisToM1AngleKinFactor * pContext->m1Context.microStepsPerRad;

  return TAM_OK;
}

/**
 * @brief Set the enabled alarms for the motors
 * 
 * @param pContext      The two axis machine context struct
 * @param alarmEn       The alarm enable bit fields to set for the motors 
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetAlarmEn(TAM_Context * const pContext, const sL6470_AlarmEnRegister_t alarmEn)
{
  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_SetAlarmEn(pContext, alarmEn, alarmEn), 
    "Error setting alarm enable to %u", *((uint8_t*)&alarmEn)
  );

  return TAM_OK;
}

/**
 * @brief Reset all parameters to default params as setup in the config
 * 
 * @param pContex       The two axis machine context struct
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_SetDefaultParams(TAM_Context * const pContext)
{
  LOG_DEBUG("Preparing to reset all TAM parameters");

  TAM_CHECK_STATUS_LOG_RETURN(TAM_Motor_SetDefaultParams(pContext), "Error setting default motor params");

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetStepSel(pContext, pContext->m0Context.pConfig->stepSel, pContext->m1Context.pConfig->stepSel), 
    "Error reseting step selection parameter"
  );

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetAlarmEn(pContext, pContext->pConfig->alarmEn), 
    "Error reseting alarm enable parameter"
  );

  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_SetKinematicParams(pContext, &(pContext->xAxis.pConfig->initialKinematicParams), &(pContext->yAxis.pConfig->initialKinematicParams)), 
    "Error updating the axis kinematic parameters"
  );

  LOG_DEBUG("Reset all TAM parameters to config defaults");

  return TAM_OK;
}

/**
 * @brief Returns the status for each motor
 * 
 * @param pContext      The two axis machine context struct
 * @param pStatusM0     Pointer to value to save the motor 0 status into
 * @param pStatusM1     Pointer to value to save the motor 1 status into
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_GetMotorStatus(TAM_Context const * const pContext, sL6470_StatusRegister_t * const pStatusM0, sL6470_StatusRegister_t * const pStatusM1)
{
  TAM_CHECK_STATUS_LOG_RETURN(
    TAM_Motor_GetStatus(pContext, pStatusM0, pStatusM1), 
    "Error getting motor status"
  );

  return TAM_OK;
}