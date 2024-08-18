#include "two_axis_machine/tam_motor_commands.h"
#include "two_axis_machine/tam_context.h"
#include "xnucleoihm02a1/microstepping_motor.h"
#include "math.h"
#include "string.h"

#define TAM_DriverHandle_M0(pContext) ((pContext)->m0Context.pDriverHandle)
#define TAM_DriverHandle_M1(pContext) ((pContext)->m1Context.pDriverHandle)

#define TAM_Command_M0(pContext) (TAM_DriverHandle_M0((pContext))->Command)
#define TAM_Command_M1(pContext) (TAM_DriverHandle_M1((pContext))->Command)
#define TAM_PerformPreparedCommands(pContext) ((pContext)->pBoardHandle->Command->PerformPreparedApplicationCommand())

#define TAM_Register_M0(pContext) (TAM_DriverHandle_M0((pContext))->Register)
#define TAM_Register_M1(pContext) (TAM_DriverHandle_M1((pContext))->Register)

static inline eL6470_DirId_t TAM_PosToDir(float pos)
{   
    if (pos < 0)
    {
        return L6470_DIR_REV_ID;
    }

    return L6470_DIR_FWD_ID;
}

/**
 * @brief Make both motors go to given absolute positions
 * 
 * @param pContext      The two axis machine context struct      
 * @param posM0         The absolute position to move motor 0 to in microsteps.  
 * @param posM1         The absolute position to move motor 1 to in microsteps.
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_Motor_GoTo(TAM_Context const * const pContext, float posM0, float posM1)
{
    LOG_DEBUG("Preparing to move M0 to %f uSteps and M1 to %f uSteps", posM0, posM1);

    TAM_Command_M0(pContext)->PrepareGoTo(
        pContext->m0Context.driverId, 
        (uint32_t)fabs(posM0)
    );

    TAM_Command_M1(pContext)->PrepareGoTo(
        pContext->m1Context.driverId,
        (uint32_t)fabs(posM1)
    );

    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Moving M0 to %f uSteps and M1 to %f uSteps", posM0, posM1);

    return TAM_OK;
}

/**
 * @brief Moves both motors for a given displacement
 * 
 * @param pContext      The two axis machine context struct 
 * @param posM0         The displacement to move motor 0 for in microsteps.
 * @param posM1         The displacement to move motor 1 for in microsteps.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_Move(TAM_Context const * const pContext, float posM0, float posM1)
{
    LOG_DEBUG("Preparing to move M0 %f uSteps and M1 %f uSteps", posM0, posM1);

    const uint32_t microStepsM0 = (uint32_t)fabs(posM0); 
    TAM_Command_M0(pContext)->PrepareMove(
        pContext->m0Context.driverId, 
        TAM_PosToDir(posM0),
        microStepsM0
    );

    const uint32_t microStepsM1 = (uint32_t)fabs(posM1);
    TAM_Command_M1(pContext)->PrepareMove(
        pContext->m1Context.driverId,
        TAM_PosToDir(posM1), 
        microStepsM1
    );

    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Moving M0 %u uSteps and M1 %u uSteps", microStepsM0, microStepsM1);

    return TAM_OK;
}

/**
 * @brief Moves both motors at a given velocity.
 * 
 * @param pContext      The two axis machine context struct 
 * @param speedM0       The velocity to move motor 0 at in full steps/s.
 * @param speedM1       The velocity to move motor 1 at in full steps/s.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_Run(TAM_Context const * const pContext, float speedM0, float speedM1)
{
    LOG_DEBUG("Preparing to run M0 at %f steps/s and M1 at %f steps/s", speedM0, speedM1);

    const uint32_t speedValM0 = Step_s_2_Speed(fabs(speedM0));
    ASSERT_CHECK_LOG_RETURN(speedValM0 != 0 || speedM0 == 0, TAM_INVALID_PARAM,
        "Invalid M0 speed %f exceeds maximum", speedM0
    );
    
    const uint32_t speedValM1 = Step_s_2_Speed(fabs(speedM1));
    ASSERT_CHECK_LOG_RETURN(speedValM1 != 0 || speedM1 == 0, TAM_INVALID_PARAM, 
        "Invalid M0 speed %f exceeds maximum", speedM1
    );

    TAM_Command_M0(pContext)->PrepareRun(
        pContext->m0Context.driverId, 
        TAM_PosToDir(speedM0),
        speedValM0
    );

    TAM_Command_M1(pContext)->PrepareRun(
        pContext->m1Context.driverId,
        TAM_PosToDir(speedM1), 
        speedValM1
    );

    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Running M0 at %f steps/s and M1 at %f steps/s", speedM0, speedM1);

    return TAM_OK;
}

/**
 * @brief Set the home position for both motors
 * 
 * @param pContext      The two axis machine context struct 
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetHome(TAM_Context const * const pContext)
{
    LOG_DEBUG("Preparing to set current position as home for M0 and M1");

    TAM_Command_M0(pContext)->PrepareResetPos(pContext->m0Context.driverId);
    TAM_Command_M1(pContext)->PrepareResetPos(pContext->m1Context.driverId);
    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Set current position as home for M0 and M1");

    return TAM_OK;
}

/**
 * @brief Move both motors to home position
 * 
 * @param pContext      The two axis machine context struct 
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_GoHome(TAM_Context const * const pContext)
{
    LOG_DEBUG("Preparing to make M0 and M1 go to home position");

    TAM_Command_M0(pContext)->PrepareGoHome(pContext->m0Context.driverId);
    TAM_Command_M1(pContext)->PrepareGoHome(pContext->m1Context.driverId);
    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Making M0 and M1 go to home position");

    return TAM_OK;
}

/**
 * @brief Soft stop both motors
 * 
 * @param pContext      The two axis machine context struct 
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SoftStop(TAM_Context const * const pContext)
{
    LOG_DEBUG("Preparing to soft stop M0 and M1");

    TAM_Command_M0(pContext)->PrepareSoftStop(pContext->m0Context.driverId);
    TAM_Command_M1(pContext)->PrepareSoftStop(pContext->m1Context.driverId);
    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Soft stopping M0 and M1");

    return TAM_OK;
}

/**
 * @brief Hard stop both motors
 * 
 * @param pContext      The two axis machine context struct 
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_HardStop(TAM_Context const * const pContext)
{
    LOG_DEBUG("Preparing to hard stop M0 and M1");

    TAM_Command_M0(pContext)->PrepareHardStop(pContext->m0Context.driverId);
    TAM_Command_M1(pContext)->PrepareHardStop(pContext->m1Context.driverId);
    TAM_PerformPreparedCommands(pContext);

    LOG_INFO("Hard stopping M0 and M1");

    return TAM_OK;
}

/**
 * @brief Get a param with given id from both motor drivers
 * 
 * @param pContext      The two axis machine context struct 
 * @param regId         The id of the parameter to retrieve.
 * @param paramM0       Pointer to value to save the motor 0 parameter into.
 * @param paramM1       Pointer to value to save the motor 1 parameter into.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_GetParam(TAM_Context const * const pContext, const eL6470_RegId_t regId, uint32_t * const paramM0, uint32_t * const paramM1)
{
    LOG_DEBUG("Preparing to get param with id %d from M0 and M1", regId);

    TAM_Command_M0(pContext)->PrepareGetParam(pContext->m0Context.driverId, regId);
    TAM_Command_M1(pContext)->PrepareGetParam(pContext->m1Context.driverId, regId);
    uint8_t * const pReturnData = TAM_PerformPreparedCommands(pContext);

    *paramM0 = L6470_ExtractReturnedData(pContext->m0Context.driverId, pReturnData, L6470_Register[regId].LengthByte);
    *paramM1 = L6470_ExtractReturnedData(pContext->m1Context.driverId, pReturnData, L6470_Register[regId].LengthByte);

    LOG_INFO("Obtained param (id %d) value %u for M0 and %u for M1", regId, *paramM0, *paramM1);

    return TAM_OK;
}

/**
 * @brief Retrieve the status for both motors
 * 
 * @param pContext      The two axis machine context struct 
 * @param pStatusM0     Pointer to value to save motor 0 status to.
 * @param pStatusM1     Pointer to value to save motor 1 status to.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_GetStatus(TAM_Context const * const pContext, sL6470_StatusRegister_t * const pStatusM0, sL6470_StatusRegister_t * const pStatusM1)
{
    LOG_DEBUG("Preparing to get M0 and M1 status");

    TAM_Command_M0(pContext)->PrepareGetStatus(pContext->m0Context.driverId);
    TAM_Command_M1(pContext)->PrepareGetStatus(pContext->m1Context.driverId);

    uint8_t * const pReturnData = TAM_PerformPreparedCommands(pContext);

    *((uint16_t*)pStatusM0) = L6470_ExtractReturnedData(pContext->m0Context.driverId, pReturnData, L6470_Register[L6470_STATUS_ID].LengthByte);
    *((uint16_t*)pStatusM1) = L6470_ExtractReturnedData(pContext->m1Context.driverId, pReturnData, L6470_Register[L6470_STATUS_ID].LengthByte);

    LOG_INFO("Obtained %u status for M0 and %u status for M1", *pStatusM0, *pStatusM1);

    return TAM_OK;
}

/**
 * @brief Set a parameter with id for both motors
 * 
 * @param pContext      The two axis machine context struct 
 * @param regId         The register id to save parameters for
 * @param paramM0       The param value to set for motor 0.
 * @param paramM1       The param value to set for motor 1.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetParam(TAM_Context * const pContext, const eL6470_RegId_t regId, const uint32_t paramM0, const uint32_t paramM1)
{
    LOG_DEBUG("Preparing to set param with id %d for M0 to %u and M1 to %u", regId, paramM0, paramM1);

    TAM_Command_M0(pContext)->PrepareSetParam(pContext->m0Context.driverId, regId, paramM0); 
    TAM_Command_M1(pContext)->PrepareSetParam(pContext->m1Context.driverId, regId, paramM1);
    TAM_PerformPreparedCommands(pContext); 

    LOG_INFO("Set param with id %d for M0 to %u and M1 to %u", regId, paramM0, paramM1);

    return TAM_OK;
}

/**
 * @brief Sets the acceleration for both motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param accM0         The motor 0 acceleration in full steps/s^2.
 * @param accM1         The motor 1 acceleration in full steps/s^2.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetAcc(TAM_Context * const pContext, const float accM0, const float accM1)
{
    LOG_DEBUG("Preparing to set the acceleration for M0 to %f and M1 to %f", accM0, accM1);

    ASSERT_CHECK_LOG_RETURN(accM0 > 0, TAM_INVALID_PARAM,
        "Invalid zero or negative acceleration for x-axis: %f", accM0
    );

    ASSERT_CHECK_LOG_RETURN((accM1 > 0), TAM_INVALID_PARAM,
        "Invalid zero or negative acceleration for y-axis: %f", accM1
    );

    const uint16_t accValM0 = Step_s2_2_Acc(accM0);
    ASSERT_CHECK_LOG_RETURN(accValM0 != 0, TAM_INVALID_PARAM,
        "Invalid acceleration %f for motor 0 exceeds minimum allowed", accM0
    );

    const uint16_t accValM1 = Step_s2_2_Acc(accM1);
    ASSERT_CHECK_LOG_RETURN(accValM1 != 0, TAM_INVALID_PARAM,
        "Invalid acceleration %f for motor 1 exceeds minimum allowed", accM1
    );
    
    pContext->m0Context.params.acc = accM0;
    TAM_Register_M0(pContext).ACC = accValM0;

    pContext->m1Context.params.acc = accM1;
    TAM_Register_M1(pContext).ACC = accValM1;

    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParam(pContext, L6470_ACC_ID, accValM0, accValM1),
        "Error setting the acceleration parameter"
    );

    LOG_INFO("Set the acceleration for M0 to %f and M1 to %f", accM0, accM1);

    return TAM_OK;
}

/**
 * @brief Sets the deceleration for both motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param decM0         The motor 0 deceleration in full steps/s^2.
 * @param decM1         The motor 1 deceleration in full steps/s^2.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetDec(TAM_Context * const pContext, const float decM0, const float decM1)
{
    LOG_DEBUG("Preparing to set deceleration for M0 to %f steps/s^2 and M1 to %f steps/s^2", decM0, decM1);

    ASSERT_CHECK_LOG_RETURN(decM0 > 0, TAM_INVALID_PARAM,
        "Invalid zero or negative deceleration for M0: %f", decM0
    );

    ASSERT_CHECK_LOG_RETURN(decM1 > 0, TAM_INVALID_PARAM,
        "Invalid zero or negative deceleration for M1: %f", decM1
    );

    const uint16_t decValM0 = Step_s2_2_Dec(decM0);
    ASSERT_CHECK_LOG_RETURN(decValM0 != 0, TAM_INVALID_PARAM,
        "Invalid deceleration %f for motor 0 exceeds minimum allowed", decM0
    );

    const uint16_t decValM1 = Step_s2_2_Dec(decM1);
    ASSERT_CHECK_LOG_RETURN(decValM1 != 0, TAM_INVALID_PARAM,
        "Invalid deceleration %f for motor 1 exceeds minimum allowed", decM1
    );

    pContext->m0Context.params.dec = decM0;
    TAM_Register_M0(pContext).DEC = decValM0;

    pContext->m1Context.params.dec = decM1;
    TAM_Register_M1(pContext).DEC = decValM1;

    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParam(pContext, L6470_DEC_ID, decValM0, decValM1),
        "Error setting the deceleration parameter"
    );

    LOG_INFO("Set deceleration for M0 to %f steps/s^2 and M1 to %f steps/s^2", decM0, decM1);

    return TAM_OK;
}

/**
 * @brief Sets the max speed for both motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param maxSpeedM0    The motor 0 max speed in full steps/s.
 * @param maxSpeedM1    The motor 1 max speed in full steps/s.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetMaxSpeed(TAM_Context * const pContext, const float maxSpeedM0, const float maxSpeedM1)
{
    LOG_DEBUG("Preparing to set the max speed for M0 to %f steps/s and M1 to %f steps/s", maxSpeedM0, maxSpeedM1);

    ASSERT_CHECK_LOG_RETURN(maxSpeedM0 > 0, TAM_INVALID_PARAM,
        "Invalid zero or negative maximum speed for M0: %f", maxSpeedM0
    );

    ASSERT_CHECK_LOG_RETURN(maxSpeedM1 > 0, TAM_INVALID_PARAM,
        "Invalid zero or negative maximum speed for M1: %f", maxSpeedM1
    );

    const uint16_t maxSpeedValM0 = Step_s_2_MaxSpeed(maxSpeedM0);
    ASSERT_CHECK_LOG_RETURN(maxSpeedValM0 != 0, TAM_INVALID_PARAM,
        "Invalid maximum speed %f for motor 0 exceeds minimum allowed", maxSpeedM0
    );

    const uint16_t maxSpeedValM1 = Step_s_2_MaxSpeed(maxSpeedM1);
    ASSERT_CHECK_LOG_RETURN(maxSpeedValM1 != 0, TAM_INVALID_PARAM,
        "Invalid maximum speed %f for motor 1 exceeds minimum allowed", maxSpeedM1
    );

    pContext->m0Context.params.maxspeed = maxSpeedM0;
    TAM_Register_M0(pContext).MAX_SPEED = maxSpeedValM0;

    pContext->m1Context.params.maxspeed = maxSpeedM1;
    TAM_Register_M1(pContext).MAX_SPEED = maxSpeedValM1;

    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParam(pContext, L6470_MAX_SPEED_ID, maxSpeedValM0, maxSpeedValM1),
        "Error setting the max speed parameter"
    );

    LOG_INFO("Set the max speed for M0 to %f steps/s and M1 to %f steps/s", maxSpeedM0, maxSpeedM1);

    return TAM_OK;
}

/**
 * @brief Sets the min speed for both motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param minSpeedM0    The motor 0 min speed in full steps/s.
 * @param minSpeedM1    The motor 1 min speed in full steps/s.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetMinSpeed(TAM_Context * const pContext, const float minSpeedM0, const float minSpeedM1)
{
    LOG_DEBUG("Preparing to set the MIN speed for M0 to %f steps/s and M1 to %f steps/s", minSpeedM0, minSpeedM1);

    ASSERT_CHECK_LOG_RETURN(minSpeedM0 > 0, TAM_INVALID_PARAM,
        "Invalid zero negative minimum speed for M0: %f", minSpeedM0
    );

    ASSERT_CHECK_LOG_RETURN(minSpeedM1 > 0, TAM_INVALID_PARAM,
        "Invalid zero negative minimum speed for M1: %f", minSpeedM1
    );

    const uint16_t minSpeedValM0 = Step_s_2_MinSpeed(minSpeedM0);
    ASSERT_CHECK_LOG_RETURN(minSpeedValM0 != 0, TAM_INVALID_PARAM,
        "Invalid minimum speed %f for motor 0 exceeds minimum allowed", minSpeedM0
    );

    const uint16_t minSpeedValM1 = Step_s_2_MinSpeed(minSpeedM1);
    ASSERT_CHECK_LOG_RETURN(minSpeedValM1 != 0, TAM_INVALID_PARAM,
        "Invalid minimum speed %f for motor 1 exceeds minimum allowed", minSpeedM1
    );
    
    pContext->m0Context.params.minspeed = minSpeedM0;
    TAM_Register_M0(pContext).MIN_SPEED = minSpeedValM0;

    pContext->m1Context.params.minspeed = minSpeedM1;
    TAM_Register_M1(pContext).MIN_SPEED = minSpeedValM1;

    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParam(pContext, L6470_MIN_SPEED_ID, minSpeedValM0, minSpeedValM1),
        "Error setting the min speed parameter"
    );

    LOG_INFO("Set the min speed for M0 to %f steps/s and M1 to %f steps/s", minSpeedM0, minSpeedM1);

    return TAM_OK;
}

/**
 * @brief Sets the step mode selection for both motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param stepSelM0     The step mode for motor 0.
 * @param stepSelM1     The step mode for motor 1.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetStepSel(TAM_Context * const pContext, const eMotorStepMode_t stepSelM0, const eMotorStepMode_t stepSelM1)
{
    LOG_DEBUG("Preparing to set the step selection value for M0 to %d and M1 to %d", stepSelM0, stepSelM1);

    pContext->m0Context.params.step_sel = stepSelM0;
    TAM_Register_M0(pContext).STEP_MODE = stepSelM0;

    pContext->m1Context.params.step_sel = stepSelM1;
    TAM_Register_M1(pContext).STEP_MODE = stepSelM1;

    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParam(pContext, L6470_STEP_MODE_ID, stepSelM0, stepSelM1),
        "Error setting the min speed parameter"
    );

    LOG_INFO("Set the step selection value for M0 to %d and M1 to %d", stepSelM0, stepSelM1);

    return TAM_OK;
}

/**
 * @brief Sets the alarm enable bits for the motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param alarmEnM0     The alarm enable bitmap for motor 0.
 * @param alarmEnM1     The alarm enable bitmap for motor 1.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetAlarmEn(TAM_Context * const pContext, const sL6470_AlarmEnRegister_t alarmEnM0, const sL6470_AlarmEnRegister_t alarmEnM1)
{
    pContext->m0Context.params.alarmen = *((uint8_t*)(&alarmEnM0));
    TAM_Register_M0(pContext).ALARM_EN = pContext->m0Context.params.alarmen;

    pContext->m1Context.params.alarmen = *((uint8_t*)(&alarmEnM1));
    TAM_Register_M1(pContext).ALARM_EN = pContext->m0Context.params.alarmen;

    LOG_DEBUG("Preparing to set the alarm enable value for M0 to %d and M1 to %d", 
        pContext->m0Context.params.alarmen,  pContext->m1Context.params.alarmen);

    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParam(pContext, L6470_ALARM_EN_ID, pContext->m0Context.params.alarmen, pContext->m1Context.params.alarmen),
        "Error setting the min speed parameter"
    );

    LOG_INFO("Set the alarm enable value for M0 to %d and M1 to %d", 
        pContext->m0Context.params.alarmen,  pContext->m1Context.params.alarmen);

    return TAM_OK;
}

/**
 * @brief Sets the set of parameters for the motors.
 * 
 * @param pContext      The two axis machine context struct 
 * @param pParamsM0     The motor 0 parameters.
 * @param pParamsM1     The motor 1 parameters.
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetParams(TAM_Context * const pContext, MotorParameterData_t const * const pParamsM0, MotorParameterData_t const * const pParamsM1)
{
    LOG_DEBUG("Preparing to set default global motor parameters");

    memcpy(&(pContext->m0Context.params), pParamsM0, sizeof(MotorParameterData_t));
    memcpy(&(pContext->m1Context.params), pParamsM1, sizeof(MotorParameterData_t));

    pContext->m0Context.pDriverHandle->Config(&pContext->m0Context.params);
    pContext->m1Context.pDriverHandle->Config(&pContext->m1Context.params);

    LOG_INFO("Set motor parameters");

    return TAM_OK;
}

/**
 * @brief Sets the default "driver" parameters for the motors
 * 
 * @param pContext      The two axis machine context struct 
 * @return TAM_Status   Two axis machine error status 
 */
TAM_Status TAM_Motor_SetDefaultParams(TAM_Context * const pContext)
{
    LOG_DEBUG("Preparing to set default global motor parameters");
    MotorParameterData_t const * const pGlobalInitData = GetMotorParameterInitData();
    
    TAM_CHECK_STATUS_LOG_RETURN(
        TAM_Motor_SetParams(pContext, &pGlobalInitData[0], &pGlobalInitData[1]),
        "Error setting params"
    );

    LOG_INFO("Set default global motor parameters");

    return TAM_OK;
}