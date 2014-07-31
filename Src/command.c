/**
  * COPYRIGHT(c) 2014 Y.Magara
  */

#include <string.h>
#include "stm32f0xx_hal.h"
#include <string.h>
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "command.h"

#define MSG_CRLF "\r\n"
#define MSG_EMPTY_ARGUMENT "Empty argument.\r\n"
#define MAG_INVALID_PARAMETER "Invalid parameter.\r\n"

/* Send Data over USART are stored in this buffer       */
static UserBufferDef UserTxBuffer[TX_BUFFER_COUNT];
static uint16_t idxTxBuffer = 0;

static CommandBufferDef CmdBuf[MAX_CMD_BUF_COUNT];
static uint16_t currentCmdIdx;

static const uint8_t HexChr[] = "0123456789ABCDEF";

static void cmdVersion(CommandBufferDef *cmd);
static void cmdPutOn(CommandBufferDef *cmd);
static void cmdTakeOff(CommandBufferDef *cmd);
static void cmdHelp(CommandBufferDef *cmd);


typedef struct  {
	const char *const name;
	void (*const func)(CommandBufferDef *cmd);
} CommandOp;

static const CommandOp CmdDic[] = {
	{"PUTON", cmdPutOn},
	{"TAKEOFF", cmdTakeOff},
	{"HELP", cmdHelp},
	{"VERSION", cmdVersion},
	{NULL, NULL}
};

/**
 * Print a string via VCP TX port.
 */
static void PutStr(char *str)
{
	uint32_t len = strlen(str);
	UserBufferDef *txBufPtr = &UserTxBuffer[idxTxBuffer];
	strlcpy((char *)txBufPtr->Buffer, str, MAX_COMMAND_LENGTH);
	txBufPtr->Length = len;
	uint16_t retry = 1000;
	while (HAL_UART_Transmit_IT(&huart1, txBufPtr->Buffer, len) == HAL_BUSY && retry-- > 0) {
		osDelay(1);
	}
	idxTxBuffer = (idxTxBuffer + 1) % TX_BUFFER_COUNT;
}

/**
 * Print a character via VCP TX port.
 */
static void PutChr(char c)
{
	UserBufferDef *txBufPtr = &UserTxBuffer[idxTxBuffer];
	txBufPtr->Buffer[0] = c;
	txBufPtr->Length = 1;
	uint16_t retry = 1000;
	while (HAL_UART_Transmit_IT(&huart1, txBufPtr->Buffer, 1) == HAL_BUSY && retry-- > 0) {
		osDelay(1);
	}
	idxTxBuffer = (idxTxBuffer + 1) % TX_BUFFER_COUNT;
}

static void PutUint16(uint16_t value)
{
	for (int s = 12; s >= 0; s -= 4) {
		PutChr(HexChr[0x0F & (value >> s)]);
	}
}

/**
  * Print version number.
  */
static void cmdVersion(CommandBufferDef *cmd)
{
	PutStr(VERSION_STR);
	PutStr(MSG_CRLF);
}


/**
  * Put a card on the RF antenna.
	*
	* PUTON <A/B/C/D>
  */
static void cmdPutOn(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
}

/**
  * Take a card off from the RF antenna.
	*
	* TAKEOFF <A/B/C/D>
  */
static void cmdTakeOff(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
}

/**
  * Show command help.
  */
static void cmdHelp(CommandBufferDef *cmd)
{
	PutStr("Show command help.\r\n");
}

/**
  * Split command and argument.
  */
static void SplitArg(CommandBufferDef *cmd)
{
	cmd->Arg = NULL;
	char *ptr = cmd->Buffer;
	char *tail = ptr + MAX_COMMAND_LENGTH;
	while (ptr < tail) {
		if (*ptr == '\0' || *ptr == ' ' || *ptr=='\t') {
			break;
		}
		ptr++;
	}
	cmd->CmdLength = (ptr - cmd->Buffer);
	if (*ptr != '\0') {
		while (ptr < tail) {
			ptr++;
			if (*ptr != ' ' && *ptr != '\t') {
				cmd->Arg = ptr;
				break;
			}
		}
	}
}
static void LookupCommand(CommandBufferDef *cmd)
{
	const CommandOp *cmdPtr, *matched;
	uint16_t matchCount;
	SplitArg(cmd);
	for (int len = 1; len <= cmd->CmdLength; len++) {
		cmdPtr = CmdDic;
		matchCount = 0;
		while (cmdPtr->name != NULL)
		{
			if (strncmp(cmdPtr->name, cmd->Buffer, len) == 0) {
				matchCount++;
				matched = cmdPtr;
			}
			cmdPtr++;
		}
		// check if only one command matched or not
		if (matchCount > 1) {
			continue;
		}
		else 
		{
			if (matchCount == 1 && cmd->CmdLength <= strlen(matched->name) && strncmp(matched->name, cmd->Buffer, cmd->CmdLength) == 0) {
				cmd->func = matched->func;
				osMessagePut(CmdBoxId, (uint32_t)cmd, 0);
			} else {
				PutStr("SYNTAX ERROR\r\n");
			}
			break;
		}
	}
}

/**
 * Parse input string from VCP RX port.
 */
void ParseInputChars(UserBufferDef *rxPtr)
{
	uint8_t *p = &rxPtr->Buffer[0];
	uint8_t *tail = &rxPtr->Buffer[rxPtr->Length];
	CommandBufferDef *cmdBufPtr = &CmdBuf[currentCmdIdx];
	while (p < tail && cmdBufPtr->Length < MAX_COMMAND_LENGTH) {
		switch (*p) {
			case '\r':
				// execute command
				PutStr(MSG_CRLF);
			  if (cmdBufPtr->Length > 0) {
					cmdBufPtr->Buffer[cmdBufPtr->Length] = '\0';
					LookupCommand(cmdBufPtr);
					currentCmdIdx = (currentCmdIdx + 1 ) % MAX_CMD_BUF_COUNT;
					cmdBufPtr = &CmdBuf[currentCmdIdx];
					cmdBufPtr->Length = 0;
				}
				break;
			case '\b':
				if (cmdBufPtr->Length > 0) {
					cmdBufPtr->Length--;
					PutStr("\b \b");
				}
				break;
			case ' ':
			case '\t':
				// skip space character at line top
				if (cmdBufPtr->Length == 0) {
					break;
				}
			default:
				PutChr(*p);
				// capitalize
			  if (*p >= 'a' && *p <= 'z') {
					*p = *p - ('a' - 'A');
				}
				cmdBufPtr->Buffer[cmdBufPtr->Length] = *p;
				cmdBufPtr->Length++;
		}
		p++;
	}
}

/*****END OF FILE****/
