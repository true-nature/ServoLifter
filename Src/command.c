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
static volatile uint16_t idxTxBuffer = 0;

static CommandBufferDef CmdBuf[MAX_CMD_BUF_COUNT];
static uint16_t currentCmdIdx;

static const uint8_t HexChr[] = "0123456789ABCDEF";

static void cmdVersion(CommandBufferDef *cmd);
static void cmdPutOn(CommandBufferDef *cmd);
static void cmdTakeOff(CommandBufferDef *cmd);
static void cmdClear(CommandBufferDef *cmd);
static void cmdHelp(CommandBufferDef *cmd);

typedef struct  {
	const char *const name;
	void (*const func)(CommandBufferDef *cmd);
} CommandOp;

typedef struct {
	const char *name;
	uint32_t position;
	TIM_HandleTypeDef  *htim_base;
	uint32_t channel;
} ServoActionDef;

#define SERVO_PUT_DEGREE 0
#define SERVO_TAKE_DEGREE 100
#define DEG2PULSE(deg)  ((deg-90)*10+1349)

static const CommandOp CmdDic[] = {
	{"CLEAR", cmdClear},
	{"PUTON", cmdPutOn},
	{"TAKEOFF", cmdTakeOff},
	{"HELP", cmdHelp},
	{"VERSION", cmdVersion},
	{NULL, NULL}
};

static ServoActionDef Servo[] = {
	{"A", 1349, &htim3, TIM_CHANNEL_1},
	{"B", 1349, &htim3, TIM_CHANNEL_2},
	{"C", 1349, &htim3, TIM_CHANNEL_3},
	{"D", 1349, &htim3, TIM_CHANNEL_4},
	{"RW", 1349, &htim2, TIM_CHANNEL_4},
	{NULL, 1349}
};

/**
 * Print a string to console.
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
 * Print a character to console.
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

static int16_t name2servoIndex(char c)
{
	int16_t index = -1;
	ServoActionDef *p = Servo;
	while (p->name != NULL)
	{
		if (p->name[0] == c) {
			index = (p - Servo);
			break;
		}
		p++;
	}
	return index;
}

/**
  * @param  idxSrv: Index of servo motor.
  * @param  start: Start postion.
  * @param  end: End position.
  */
static void moveServo(int16_t index, uint32_t end)
{
	ServoActionDef *servo = &Servo[index];
	 static TIM_OC_InitTypeDef sConfigOC;
	uint32_t step = (end >= servo->position ? 1 : -1);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	for (;;) {
		sConfigOC.Pulse = servo->position;
		HAL_TIM_PWM_ConfigChannel(servo->htim_base, &sConfigOC, servo->channel);
		HAL_TIM_PWM_Start(servo->htim_base, servo->channel);
		 if (servo->position == end) break;
		servo->position += step;
		//osDelay(1);
		osDelay(0);
	};

	HAL_TIM_PWM_Stop(servo->htim_base, servo->channel);
}

/**
  * Print version number.
  */
static void cmdClear(CommandBufferDef *cmd)
{
	PutStr("Clear all arms.\r\n");
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
		return;
	}
	int16_t index = name2servoIndex(cmd->Arg[0]);
	if (index < 0) {
		PutStr(MAG_INVALID_PARAMETER);
		return;
	}
	moveServo(index, DEG2PULSE(SERVO_PUT_DEGREE));
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
		return;
	}
	int16_t index = name2servoIndex(cmd->Arg[0]);
	if (index < 0) {
		PutStr(MAG_INVALID_PARAMETER);
		return;
	}
	moveServo(index, DEG2PULSE(SERVO_TAKE_DEGREE));
}

/**
  * Show command help.
  */
static void cmdHelp(CommandBufferDef *cmd)
{
	PutStr("Show command help.\r\n");
}
void StartMotorThread(void const * argument)
{
	osEvent evt;
	CommandBufferDef *cmdBuf;
	cmdVersion(NULL);
	PutStr("OK\r\n");
//	cmdClear();
  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(CmdBoxId, osWaitForever);
		if (evt.status == osEventMessage) {
			cmdBuf = evt.value.p;
			cmdBuf->func(cmdBuf);
			PutStr("OK\r\n");
		}
		//check received length, read UserRxBufferFS
  }
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
void ParseInputChars(uint8_t ch)
{
	CommandBufferDef *cmdBufPtr = &CmdBuf[currentCmdIdx];
	switch (ch) {
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
			PutChr(ch);
			// capitalize
			if (ch >= 'a' && ch <= 'z') {
				ch = ch - ('a' - 'A');
			}
			cmdBufPtr->Buffer[cmdBufPtr->Length] = ch;
			cmdBufPtr->Length++;
	}
}

/*****END OF FILE****/
