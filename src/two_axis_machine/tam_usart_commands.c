
#include "two_axis_machine/tam_usart_commands.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "stm32f4xx_hal_usart.h"
#include "trace.h"
#include "two_axis_machine/tam_context.h"
#include "two_axis_machine/tam_commands.h"

#define MAX_COMMAND_LENGTH 12
#define MAX_PARAM_LENGTH 12
#define MAX_PARAMS 2

/**
 * @brief The USART command bundle decoded from USART input
 */
typedef struct {
  char commandName[MAX_COMMAND_LENGTH];  //!< command mnemonic
  char params[2][MAX_PARAM_LENGTH];     //!< parameters of command
} TAM_USART_TextCommandBundle;

int strcasecmp(const char *string1, const char *string2) 
{
	// Compare each character of the strings
	while (*string1 && *string2) 
	{
		char c1 = tolower((unsigned char)*string1);
		char c2 = tolower((unsigned char)*string2);

		if (c1 != c2) 
		{
			return (c1 - c2);
		}
		string1++;
		string2++;
	}

	// If we reach here, one or both strings have ended
	return (tolower((unsigned char)*string1) - tolower((unsigned char)*string2));
}

/**
 * @brief Splits and decodes the input command string into the command bundle
 * 
 * @param pTextString 				The text string to decode
 * @param pTextCommandBundle 	The command bundle to initialize
 * @return TAM_Status 				Two axis machine error status
 */
TAM_Status TAM_USART_SplitTextString(char const * const pTextString, TAM_USART_TextCommandBundle * const pTextCommandBundle) 
{
	const char * const COMMAND_DELIM = "_- ";
	memset(pTextCommandBundle, '\0', sizeof(TAM_USART_TextCommandBundle));

	// Copy the input string to a temporary buffer to tokenize
	char buffer[MAX_COMMAND_LENGTH];
	strncpy(buffer, pTextString, MAX_COMMAND_LENGTH - 1);
	buffer[MAX_COMMAND_LENGTH - 1] = '\0'; // Ensure null termination

	char *pToken = strtok(buffer, COMMAND_DELIM);

	// Extract the command
	if (pToken != NULL) 
	{
		strncpy(pTextCommandBundle->commandName, pToken, MAX_COMMAND_LENGTH - 1);
		pTextCommandBundle->commandName[MAX_COMMAND_LENGTH - 1] = '\0'; // Ensure null termination
	}

	for (unsigned int i = 0; i < MAX_PARAMS; ++i)
	{
		pToken = strtok(NULL, COMMAND_DELIM);
		if (pToken != NULL) 
		{
			strncpy(&(pTextCommandBundle->params[i][0]), pToken, MAX_COMMAND_LENGTH - 1);
			pTextCommandBundle->params[i][MAX_COMMAND_LENGTH - 1] = '\0'; // Ensure null termination
		}
	}

	if (strtok(NULL, COMMAND_DELIM) != NULL)
	{
		PRINTLN("Too many parameters entered!\n\r");
		return TAM_INVALID_STRING_COMMAND;
	}

	return TAM_OK;
}

/**
 * @brief Converts a command parameter to float value
 * 
 * @param pParamText 		The parameter text to convert
 * @param pParamName 		The parameter name
 * @param pValue 				Pointer to the output value
 * @return TAM_Status 	The two axis machine error status
 */
TAM_Status TAM_USART_ParamToFloat(char const * const pParamText, char const * const pParamName, float* const pValue)
{
	char *endptr = NULL;

	*pValue = strtof(pParamText, &endptr);

	if (endptr == pParamText)
	{
		PRINTLN("Invalid param %s. No digits were found. Must be a floating value", pParamName);
		return TAM_INVALID_STRING_COMMAND;
	} 
	
	if (*endptr != '\0') 
	{
		PRINTLN("Invalid param %s. Further characters found: %s", pParamName, endptr);
		return TAM_INVALID_STRING_COMMAND;
	} 
	
	return TAM_OK;
}

/**
 * @brief Converts a command parameter to positive float value
 * 
 * @param pParamText 		The parameter text to convert
 * @param pParamName 		The parameter name
 * @param pValue 				Pointer to the output value
 * @return TAM_Status 	The two axis machine error status
 */
TAM_Status TAM_USART_ParamToPositiveFloat(char const * const pParamText, char const * const pParamName, float * const pValue)
{
	TAM_CHECK_STATUS_RETURN(TAM_USART_ParamToFloat(pParamText, pParamName, pValue));

	if (*pValue <= 0)
	{
		PRINTLN("Invalid param %s. Value cannot be zero or negative", pParamName);
		return TAM_INVALID_STRING_COMMAND;
	}
	
	return TAM_OK;
}

/**
 * @brief Perform the command decoded in the command bundle
 * 
 * @param pContext 						The two axis machine context
 * @param pTextCommandBundle 	The command bundle containing command to execute
 * @return TAM_Status 				The two axis machine error status
 */
TAM_Status TAM_USART_PerformCommand(TAM_Context * const pContext, TAM_USART_TextCommandBundle const * const pTextCommandBundle)
{
	
	if (strcasecmp(pTextCommandBundle->commandName, "move") == 0)
	{
		float xPos = 0, yPos = 0;
		TAM_CHECK_STATUS_RETURN(
			TAM_USART_ParamToPositiveFloat(pTextCommandBundle->params[0], "x distance", &xPos)
		);

		TAM_CHECK_STATUS_RETURN(
			TAM_USART_ParamToPositiveFloat(pTextCommandBundle->params[1], "y distance", &yPos)
		);

		PRINTLN("Performing Usart command move with x distance of %f and y distance %f", xPos, yPos);

		TAM_CHECK_STATUS_RETURN(TAM_Move(pContext, xPos, yPos));
	}
	else if (strcasecmp(pTextCommandBundle->commandName, "run") == 0)
	{
		float xSpeed = 0, ySpeed = 0;
		TAM_CHECK_STATUS_RETURN(
			TAM_USART_ParamToPositiveFloat(pTextCommandBundle->params[0], "x speed", &xSpeed)
		);


		TAM_CHECK_STATUS_RETURN(
			TAM_USART_ParamToPositiveFloat(pTextCommandBundle->params[1], "y speed", &ySpeed)
		);

		PRINTLN("Performing Usart command move with x speed of %f and y speed %f", xSpeed, ySpeed);

		TAM_CHECK_STATUS_RETURN(TAM_Run(pContext, xSpeed, ySpeed));
	} 
	else
	{
		PRINTLN("Unknown command %s", pTextCommandBundle->commandName);
		return TAM_INVALID_STRING_COMMAND;
	}

	return TAM_OK;
}

/**
 * @brief Decode and perform the input string command
 * 
 * @param pContext 						The two axis machine context
 * @param pTextCommandBundle 	The command string to decode and execute
 * @return TAM_Status 				The two axis machine error status
 */
TAM_Status TAM_USART_PerformStringCommand(TAM_Context * const pContext, char  const * const pTextString) 
{
	TAM_USART_TextCommandBundle textCommandBundle = {0};
	
	TAM_CHECK_STATUS_RETURN(TAM_USART_SplitTextString(pTextString, &textCommandBundle));
	LOG_DEBUG("Command %s entered with param 1 %s and param 2 %s", 
		textCommandBundle.commandName, textCommandBundle.params[0], textCommandBundle.params[1]);
	TAM_CHECK_STATUS_RETURN(TAM_USART_PerformCommand(pContext, &textCommandBundle));

	return TAM_OK;
}

/**
 * @brief Checks the USART for incoming data and performs the commands
 * 
 * @param huart 				The USART handle to use
 * @param pContext 			The two axis machine context
 * @return TAM_Status		The two axis machine error status
 */
TAM_Status TAM_USART_CheckAppCmd(UART_HandleTypeDef* huart, TAM_Context * const pContext) 
{
	static char commandText[64];

	/* Checks the UART2 is in idle state */
	if (huart->gState == HAL_UART_STATE_READY) 
	{
		/* Checks one character has been at least entered */
		if (commandText[0] != '\0') 
		{
			/* Decode the entered command string */
			TAM_Status status = TAM_USART_PerformStringCommand(pContext, commandText);
			if (status != TAM_OK && status != TAM_INVALID_STRING_COMMAND)
			{
				PRINTLN("Unknown error performing string command");
			}
		}

		/* Prepare to receive a text string via USART with UART_IT_RXNE */
		HAL_UART_Receive_IT(huart, (uint8_t*)commandText, sizeof(commandText));
	}

	return TAM_OK;
}

