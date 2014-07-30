/**
  * COPYRIGHT(c) 2014 Y.Magara
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMAND_H
#define __COMMAND_H
#include "cmsis_os.h"

#define VERSION_STR "ServoLifter version 0.1"

#define MAX_COMMAND_LENGTH 127
#define MAX_CMD_BUF_COUNT	3
typedef struct CommandBufferDef {
	uint32_t Length;
	char Buffer[MAX_COMMAND_LENGTH + 1];
	uint32_t CmdLength;
	char *Arg;
	void (*func)(struct CommandBufferDef *cmd);
} CommandBufferDef;

// card position index
typedef enum {
	Index_A = 0,
	Index_B,
	Index_C,
	Index_D,
	Index_E
} ServoIndex;

#define PWM_ARM_UP (1500-1)
#define PWM_ARM_DOWN (1300-1)
#define SERVO_WAIT_DEFAULT_MS 400

extern osMessageQId  CmdBoxId;

//extern void ParseInputChars(UsbUserBufferDef *rxPtr);
extern void StartMotorThread(void const * argument);

#endif /* __COMMAND_H */
