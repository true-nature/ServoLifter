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
#define MSG_INVALID_PARAMETER "Invalid parameter.\r\n"
#define MSG_ALRELADY_PUT "Already put on.\r\n"
#define MSG_BEAM_EMPTY "No beam is put on.\r\n"


/* Send Data over USART are stored in this buffer       */
static UserBufferDef UserTxBuffer[TX_BUFFER_COUNT];
static volatile uint16_t idxTxBuffer = 0;

static CommandBufferDef CmdBuf[MAX_CMD_BUF_COUNT];
static uint16_t currentCmdIdx;

static void cmdVersion(CommandBufferDef *cmd);
static void cmdPutOn(CommandBufferDef *cmd);
static void cmdTakeOff(CommandBufferDef *cmd);
static void cmdClear(CommandBufferDef *cmd);
static void cmdNeutral(CommandBufferDef *cmd);
static void cmdHelp(CommandBufferDef *cmd);

typedef struct  {
	const char *const name;
	void (*const func)(CommandBufferDef *cmd);
} CommandOp;

typedef struct {
	char *name;
	TIM_HandleTypeDef  *htim_base;
	uint32_t channel;
	__IO uint32_t position;
	__IO uint32_t goal;
} ServoActionDef;

#define SERVO_PUT_DEGREE (45)
#define SERVO_TAKE_DEGREE (-45)
#define SERVO_NEUTRAL_DEGREE 0
#define DEG2PULSE(deg)  (1600+9*(deg))
#define SERVO_PERIOD_MS 20

static const CommandOp CmdDic[] = {
	{"CLEAR", cmdClear},
	{"PUTON", cmdPutOn},
	{"TAKEOFF", cmdTakeOff},
	{"HELP", cmdHelp},
	{"VERSION", cmdVersion},
	{"NEUTRAL", cmdNeutral},
	{NULL, NULL}
};

static ServoActionDef Servo[] = {
	{"R", &htim2, TIM_CHANNEL_4, 1499, 1499},
	{"A", &htim3, TIM_CHANNEL_1, 1499, 1499},
	{"B", &htim3, TIM_CHANNEL_2, 1499, 1499},
	{"C", &htim3, TIM_CHANNEL_3, 1499, 1499},
	{"D", &htim3, TIM_CHANNEL_4, 1499, 1499}
};
#define NUM_OF_SERVO 5

static int16_t BeamStack[NUM_OF_SERVO];
static int16_t BeamPtr;

static uint32_t ActiveChannel2Channel(HAL_TIM_ActiveChannel ac)
{
	uint32_t channel = TIM_CHANNEL_ALL;
	switch (ac)
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			channel = TIM_CHANNEL_1;
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			channel = TIM_CHANNEL_2;
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			channel = TIM_CHANNEL_3;
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			channel = TIM_CHANNEL_4;
			break;
		default:
			break;
	}
	return channel;
}

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

void PutUint16(uint16_t value)
{
  static const uint8_t HexChr[] = "0123456789ABCDEF";
	for (int s = 12; s >= 0; s -= 4) {
		PutChr(HexChr[0x0F & (value >> s)]);
	}
}

static uint8_t IsBeamPutOn(int16_t index)
{
	uint8_t result = 0;
	for (int p = 0; p < BeamPtr; p++) 
	{
		if (BeamStack[p] == index) 
		{
			result = 1;
			break;
		}
	}
	return result;
}

static void PushBeam(int16_t index)
{
	if (BeamPtr < NUM_OF_SERVO) 
	{
		BeamStack[BeamPtr++] = index;
	}
	else
	{
		PutStr("Beam stack overflow!\r\n");
	}
}

static int16_t PopBeam()
{
	if (BeamPtr > 0)
	{
		return BeamStack[--BeamPtr];
	}
	else
	{
		return -1;
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
	for (int16_t s = 0; s < NUM_OF_SERVO; s++)
	{
		if (Servo[s].name[0] == c)
		{
			index = s;
			break;
		}
	}
	return index;
}

/**
 * Re-scan current position by reading timer register.
 */
static void RescanPosition()
{
	for (int index = 0; index < NUM_OF_SERVO; index++)
	{
		Servo[index].position = __HAL_TIM_GetCompare(Servo[index].htim_base, Servo[index].channel);
	}
}

/**
  * @param  idxSrv: Index of servo motor.
  * @param  start: Start postion.
  * @param  end: End position.
  */
static void moveServo(int16_t index, uint32_t goal)
{
	ServoActionDef *servo = &Servo[index];
	servo->goal = goal;
	HAL_TIM_PWM_Start_IT(servo->htim_base, servo->channel);
	while (servo->position != goal)
	{
		osDelay(1);
	}
	// stop callback
	osDelay(2*SERVO_PERIOD_MS);
	HAL_TIM_PWM_Stop_IT(servo->htim_base, servo->channel);
	osDelay(SERVO_PERIOD_MS);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}


/**
  * Clear all arms.
  */
static void cmdClear(CommandBufferDef *cmd)
{
	// set beam stack by order of position
	RescanPosition();
	BeamPtr = 0;
	int16_t *p, *q;
	int16_t *tail = &BeamStack[NUM_OF_SERVO];
	for (uint16_t index = 0; index < NUM_OF_SERVO; index++)
	{
		BeamStack[BeamPtr++] = NUM_OF_SERVO - 1 - index;
	}
	for (p = BeamStack; p < tail; p++)
	{
		for (q = p + 1; q < tail; q++)
		{
			if (Servo[*p].position < Servo[*q].position)
			{
				int16_t tmp = *q;
				*q = *p;
				*p = tmp;
			}
		}
	}
	// unstack all
	PutStr("CLEAR ");
	for (;;)
	{
		int16_t index = PopBeam();
		if (index < 0) {
			break;
		}
		PutChr(Servo[index].name[0]);
		PutChr(' ');
		moveServo(index, DEG2PULSE(SERVO_TAKE_DEGREE));
	}
	PutStr("\r\n");
}

/**
  * Move arms to neutral position of servo.
  */
static void cmdNeutral(CommandBufferDef *cmd)
{
	cmdClear(NULL);
	PutStr("NEUTRAL ");
	for (uint16_t index = 0; index < NUM_OF_SERVO; index++)
	{
		PutChr(Servo[index].name[0]);
		PutChr(' ');
		moveServo(index, DEG2PULSE(SERVO_NEUTRAL_DEGREE));
	}
	PutStr("\r\n");
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
	if (!IsBeamPutOn(index)) 
	{
		if (index < 0) {
			PutStr(MSG_INVALID_PARAMETER);
			return;
		}
		moveServo(index, DEG2PULSE(SERVO_PUT_DEGREE) - BeamPtr);
		PushBeam(index);
	}
	else
	{
		PutStr(MSG_ALRELADY_PUT);
	}
}

/**
  * Take a card off from the RF antenna.
	*
	* TAKEOFF
  */
static void cmdTakeOff(CommandBufferDef *cmd)
{
	if (cmd->Arg != NULL) {
		PutStr(MSG_INVALID_PARAMETER);
		return;
	}
	int16_t index = PopBeam();
	if (index < 0) {
		PutStr(MSG_BEAM_EMPTY);
		return;
	}
	PutStr("TAKEOFF ");
	PutStr(Servo[index].name);
	PutStr("\r\n");
	moveServo(index, DEG2PULSE(SERVO_TAKE_DEGREE));
}

/**
  * Show command help.
  */
static void cmdHelp(CommandBufferDef *cmd)
{
	PutStr("HELP\r\n  Show command help.\r\n");
	PutStr("VERSION\r\n  Show version string.\r\n");
	PutStr("PUTON <A|B|C|D|R>\r\n  Put a card or the Reader to the target.\r\n");
	PutStr("TAKEOFF\r\n  Take a card or the Reader from the target.\r\n");
	PutStr("CLEAR\r\n  Take all cards and the Reader from the target.\r\n");
	PutStr("NEUTRAL\r\n  Move all servo motors to neutral position.\r\n");
}

void StartMotorThread(void const * argument)
{
	osEvent evt;
	CommandBufferDef *cmdBuf;
	osDelay(500);
	cmdVersion(NULL);
	cmdClear(NULL);
	PutStr("OK\r\n");
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

//
ServoActionDef *handle2servo(TIM_HandleTypeDef *htim)
{
	ServoActionDef *srv = NULL;
	uint32_t channel = ActiveChannel2Channel(htim->Channel) ;
	for (int s = 0; s < NUM_OF_SERVO; s++)
	{
		if (Servo[s].htim_base == htim && Servo[s].channel == channel)
		{
			srv = &Servo[s];
			break;
		}
	}
	return srv;
}

/**
  * @brief  PWM Pulse finished callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	ServoActionDef *srv = handle2servo(htim);
	if (srv != NULL)
	{
		if (srv->position < srv->goal)
		{
			srv->position +=  (srv->goal - srv->position > 50 ? 25 : 1);
		}
		else if (srv->position > srv->goal)
		{
			srv->position -= (srv->position - srv->goal > 50 ? 25 : 1);
		}
		__HAL_TIM_SetCompare(htim, srv->channel, srv->position);
		// blink LEDs
		if ((srv->position >> 5) & 1)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		}
	}
}

/*****END OF FILE****/
