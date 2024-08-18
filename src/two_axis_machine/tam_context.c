#include "two_axis_machine/tam_context.h"
#include "string.h"
#include "stdlib.h"

#define M_PI		3.14159265358979323846

/**
 * @brief Creates a two axis machine context from the config
 * 
 * @param pConfig        The two axis machine config
 * @param pContext       The pointer to two axis machine context to create. Must be allocated by user
 * @return TAM_Status    Two axis machine error status
 */
TAM_Status TAM_Create_Context(TAM_Config const * const pConfig, TAM_Context * const pContext)
{
    LOG_DEBUG("Creating a new TAM context at %X from config at %X...", pContext, pConfig);
    TAM_CHECK_STATUS_LOG_RETURN(TAM_Delete_Context(pContext), "Error uninitializing context");

    // Context
    pContext->pConfig = (TAM_Config*)malloc(sizeof(TAM_Config));
    memcpy(pContext->pConfig, pConfig, sizeof(TAM_Config));

    // Board
        pContext->boardId = EXPBRD_ID(0) + pContext->pConfig->boardPosition;
        pContext->pBoardHandle = BSP_GetExpansionBoardHandle(pContext->boardId);

    // x-axis and y-axis
    {
        TAM_Axis_Context * const pAxisContext[] = {&(pContext->xAxis), &(pContext->yAxis)};
        TAM_Axis_Config const * const pAxisConfigs[] = {&(pContext->pConfig->xAxis), &(pContext->pConfig->yAxis)};

        for (unsigned int i = 0; i < 2; ++i)
        {
            pAxisContext[i]->pConfig = pAxisConfigs[i];
            memcpy(&(pAxisContext[i]->kinematicParams), &(pAxisConfigs[i]->initialKinematicParams), sizeof(TAM_AxisKinematicParameters));
        }
    }

    // m0 and m1
    {
        TAM_Motor_Context * const pMotorContext[] = {&(pContext->m0Context), &(pContext->m1Context)};
        TAM_Motor_Config const * const pMotorConfigs[] = {&(pContext->pConfig->m0Config), &(pContext->pConfig->m1Config)};

        for (unsigned int i = 0; i < 2; ++i)
        {
            pMotorContext[i]->pConfig = pMotorConfigs[i];
            pMotorContext[i]->driverId = L6470_ID(0) + pMotorConfigs[i]->driverPosition;
            pMotorContext[i]->pDriverHandle = pContext->pBoardHandle->StepperMotorDriverHandle[pMotorContext[i]->driverId];

            pMotorContext[i]->fullStepsPerRad = pMotorConfigs[i]->fullStepsPerRev / (2 * M_PI);
        }
    }

    // Physical values
    {
        pContext->motorPulleyRadius = pContext->pConfig->beltPitch * pContext->pConfig->motorPulleyTeeth / (2 * M_PI); 
        pContext->platformPulleyRadius = pContext->pConfig->beltPitch * pContext->pConfig->motorPulleyTeeth / (2 * M_PI);
        pContext->yAxisScrewLeadRadians = pContext->pConfig->yAxisScrewLead /  (2 * M_PI);

        pContext->xAxisToM0AngleKinFactor = 1.0 / pContext->motorPulleyRadius;
        pContext->xAxisToM1AngleKinFactor = 1.0 / pContext->motorPulleyRadius;
        pContext->yAxisToM1AngleKinFactor = pContext->platformPulleyRadius / (pContext->motorPulleyRadius * pContext->yAxisScrewLeadRadians);
    
        pContext->xAxisToM0FullStepsKinFactor = pContext->xAxisToM0AngleKinFactor * pContext->m0Context.fullStepsPerRad;
        pContext->xAxisToM1FullStepsKinFactor = pContext->xAxisToM1AngleKinFactor * pContext->m1Context.fullStepsPerRad;
        pContext->yAxisToM1FullStepsKinFactor = pContext->yAxisToM1AngleKinFactor * pContext->m1Context.fullStepsPerRad;
    }

    LOG_INFO("Created a new TAM context at %X from config at %X", pContext, pConfig);

    return TAM_OK;
}

/**
 * @brief Deinitializes, cleans up and frees any resources used by the two axis machine context 
 * 
 * @param pContext       The two axis machine context to deinitialize
 * @return TAM_Status   Two axis machine error status
 */
TAM_Status TAM_Delete_Context(TAM_Context * const pContext)
{
    LOG_DEBUG("Deleting TAM context at %X", pContext);

    free(pContext->pConfig);
    memset(pContext, 0, sizeof(TAM_Context));

    LOG_INFO("Deleted TAM context at %X", pContext);

    return TAM_OK;
}