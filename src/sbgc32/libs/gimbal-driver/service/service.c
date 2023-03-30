/** ____________________________________________________________________
 *
 *	@file		service.c
 *
 *	@brief 		Service source file
 *	____________________________________________________________________
 *
 *	@attention	<center><h3>
 *	Copyright © 2022 BaseCam Electronics™.</h3></center>
 *	<center>All rights reserved.</center>
 *
 *	Licensed under the Apache License, Version 2.0 (the "License");
 *	you may not use this file except in compliance with the License.
 *	You may obtain a copy of the License at
 *
 *	http://www.apache.org/licenses/LICENSE-2.0
 *
 *	Unless required by applicable law or agreed to in writing, software
 *	distributed under the License is distributed on an "AS IS" BASIS,
 *	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *	implied. See the License for the specific language governing
 *	permissions and limitations under the License.
 *	____________________________________________________________________
 */

#include "service.h"


#ifdef	SYS_BIG_ENDIAN

	/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
	 *								Parser Big Endian Mapping Structures
	 */
	/**	@addtogroup	Board_Info
	 * 	@{
	 */
	/** @brief	Sample for Big Endian Mapping
	 *
	 * 	@ref	BoardInfo_ParserStructDB
	 */
	const BoardInfo_t boardInfo_ParserStruct;

	const ParserBlock_t BoardInfo_ParserStructDB [] =
	{
		VAR_BLOCK(boardInfo_ParserStruct.boardVer),
		VAR_BLOCK(boardInfo_ParserStruct.firmwareVer),
		VAR_BLOCK(boardInfo_ParserStruct.stateFlags1),
		VAR_BLOCK(boardInfo_ParserStruct.boardFeatures),
		VAR_BLOCK(boardInfo_ParserStruct.connectionFlag),
		VAR_BLOCK(boardInfo_ParserStruct.FRW_ExtraID),
		VAR_BLOCK(boardInfo_ParserStruct.boardFeaturesExt),
		DATA_BLOCK(boardInfo_ParserStruct.reserved),
		VAR_BLOCK(boardInfo_ParserStruct.baseFRW_Ver),
	};

	const ui8 BoardInfo_ParserStructDB_Size = countof(BoardInfo_ParserStructDB);


	/** @brief	Sample for Big Endian Mapping
	 *
	 * 	@ref	BoardInfo3_ParserStructDB
	 */
	const BoardInfo3_t boardInfo3_ParserStruct;

	const ParserBlock_t BoardInfo3_ParserStructDB [] =
	{
		DATA_BLOCK(boardInfo3_ParserStruct.deviceID),
		DATA_BLOCK(boardInfo3_ParserStruct.MCU_ID),
		VAR_BLOCK(boardInfo3_ParserStruct.EEPROM_Size),
		VAR_BLOCK(boardInfo3_ParserStruct.scriptSlot1_Size),
		VAR_BLOCK(boardInfo3_ParserStruct.scriptSlot2_Size),
		VAR_BLOCK(boardInfo3_ParserStruct.scriptSlot3_Size),
		VAR_BLOCK(boardInfo3_ParserStruct.scriptSlot4_Size),
		VAR_BLOCK(boardInfo3_ParserStruct.scriptSlot5_Size),
		VAR_BLOCK(boardInfo3_ParserStruct.profileSetSlots),
		VAR_BLOCK(boardInfo3_ParserStruct.profileSetCur),
		VAR_BLOCK(boardInfo3_ParserStruct.flashSize),
		DATA_BLOCK(boardInfo3_ParserStruct.reserved),
	};

	const ui8 BoardInfo3_ParserStructDB_Size = countof(BoardInfo3_ParserStructDB);
	/**	@}
	 */


	/**	@addtogroup	Auto_PID
	 * 	@{
	 */
	/** @brief	Sample for Big Endian Mapping
	 *
	 * 	@ref	AutoPID_ParserStructDB
	 */
	const AutoPID_t autoPID_ParserStruct;

	const ParserBlock_t AutoPID_ParserStructDB [] =
	{
		VAR_BLOCK(autoPID_ParserStruct.profileID),
		VAR_BLOCK(autoPID_ParserStruct.cfgFlags),
		VAR_BLOCK(autoPID_ParserStruct.Gain_VS_Stability),
		VAR_BLOCK(autoPID_ParserStruct.momentum),
		VAR_BLOCK(autoPID_ParserStruct.action),
		DATA_BLOCK(autoPID_ParserStruct.reserved),
	};

	const ui8 AutoPID_ParserStructDB_Size = countof(AutoPID_ParserStructDB);


	/** @brief	Sample for Big Endian Mapping
	 *
	 * 	@ref	AutoPID_2_ParserStructDB
	 */
	const AutoPID_2_t autoPID_2_ParserStruct;

	const ParserBlock_t AutoPID_2_ParserStructDB [] =
	{
		VAR_BLOCK(autoPID_2_ParserStruct.action),
		DATA_BLOCK(autoPID_2_ParserStruct.reserved1),
		VAR_BLOCK(autoPID_2_ParserStruct.cfgVersion),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].axisFlag),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].gain),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].stimulus),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].effectiveFreq),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].problemFreq),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].problemMargin),
		DATA_BLOCK(autoPID_2_ParserStruct.AxisAPID2[ROLL].reserved2),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].axisFlag),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].gain),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].stimulus),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].effectiveFreq),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].problemFreq),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].problemMargin),
		DATA_BLOCK(autoPID_2_ParserStruct.AxisAPID2[PITCH].reserved2),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].axisFlag),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].gain),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].stimulus),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].effectiveFreq),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].problemFreq),
		VAR_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].problemMargin),
		DATA_BLOCK(autoPID_2_ParserStruct.AxisAPID2[YAW].reserved2),
		VAR_BLOCK(autoPID_2_ParserStruct.generalFlags),
		VAR_BLOCK(autoPID_2_ParserStruct.startupCfg),
		DATA_BLOCK(autoPID_2_ParserStruct.reserved3),
	};

	const ui8 AutoPID_2_ParserStructDB_Size = countof(AutoPID_2_ParserStructDB);


	/** @brief	Sample for Big Endian Mapping
	 *
	 * 	@ref	AutoPID_State_ParserStructDB
	 */
	const AutoPID_State_t autoPID_State_ParserStruct;

	const ParserBlock_t AutoPID_State_ParserStructDB [] =
	{
		DATA_BLOCK(autoPID_State_ParserStruct.p),
		DATA_BLOCK(autoPID_State_ParserStruct.i),
		DATA_BLOCK(autoPID_State_ParserStruct.d),
		DATA_BLOCK(autoPID_State_ParserStruct.LPF_Freq),
		VAR_BLOCK(autoPID_State_ParserStruct.iterationCnt),
		VAR_BLOCK(autoPID_State_ParserStruct.AxisAPIDS[ROLL].trackingError),
		DATA_BLOCK(autoPID_State_ParserStruct.AxisAPIDS[ROLL].reserved1),
		VAR_BLOCK(autoPID_State_ParserStruct.AxisAPIDS[PITCH].trackingError),
		DATA_BLOCK(autoPID_State_ParserStruct.AxisAPIDS[PITCH].reserved1),
		VAR_BLOCK(autoPID_State_ParserStruct.AxisAPIDS[YAW].trackingError),
		DATA_BLOCK(autoPID_State_ParserStruct.AxisAPIDS[YAW].reserved1),
		DATA_BLOCK(autoPID_State_ParserStruct.reserved2),
	};

	const ui8 AutoPID_State_ParserStructDB_Size = countof(AutoPID_State_ParserStructDB);
	/**	@}
	 */


	/**	@addtogroup	State_Vars
	 * 	@{
	 */
	/** @brief	Sample for Big Endian Mapping
	 *
	 * 	@ref	StateVars_ParserStructDB
	 */
	const StateVars_t stateVars_ParserStruct;

	const ParserBlock_t StateVars_ParserStructDB [] =
	{
		DATA_BLOCK(stateVars_ParserStruct.stepSignalVars),
		VAR_BLOCK(stateVars_ParserStruct.subError),
		VAR_BLOCK(stateVars_ParserStruct.maxAcc),
		VAR_BLOCK(stateVars_ParserStruct.workTime),
		VAR_BLOCK(stateVars_ParserStruct.startupCnt),
		VAR_BLOCK(stateVars_ParserStruct.maxCurrent),
		VAR_BLOCK(stateVars_ParserStruct.IMU_TempMin),
		VAR_BLOCK(stateVars_ParserStruct.IMU_TempMax),
		VAR_BLOCK(stateVars_ParserStruct.MCU_TempMin),
		VAR_BLOCK(stateVars_ParserStruct.MCU_TempMax),
		DATA_BLOCK(stateVars_ParserStruct.shockCnt),
		VAR_BLOCK(stateVars_ParserStruct.energyTime),
		VAR_BLOCK(stateVars_ParserStruct.energy),
		VAR_BLOCK(stateVars_ParserStruct.AVG_CurrentTime),
		VAR_BLOCK(stateVars_ParserStruct.AVG_Current),
		DATA_BLOCK(stateVars_ParserStruct.reserved),
	};

	const ui8 StateVars_ParserStructDB_Size = countof(StateVars_ParserStructDB);
	/**	@}
	 */

#endif


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *													Executable Functions
 */
/**	@addtogroup	Board_Info
 * 	@{
 */
/**	@brief	Reads version and board information
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	*boardInfo - structure storing board information
 *	@param	cfg - configuration for this serial driver
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_ReadBoardInfo (GeneralSBGC_t *generalSBGC, BoardInfo_t *boardInfo, ui16 cfg)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_BOARD_INFO);
	WriteWord(&cmd, cfg);

	if (CheckReceipt(generalSBGC, SBGC32_TX_RX(generalSBGC, &cmd, CMD_BOARD_INFO), "Board Info:") == TX_RX_OK)
		ReadBuff(&cmd, boardInfo, sizeof(BoardInfo_t), PM_BOARD_INFO);

	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Reads additional board information
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	*boardInfo3 - structure storing additional board information
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_ReadBoardInfo3 (GeneralSBGC_t *generalSBGC, BoardInfo3_t *boardInfo3)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_BOARD_INFO_3);

	if (CheckReceipt(generalSBGC, SBGC32_TX_RX(generalSBGC, &cmd, CMD_BOARD_INFO_3), "Board Info 3:") == TX_RX_OK)
		ReadBuff(&cmd, boardInfo3, sizeof(BoardInfo3_t), PM_BOARD_INFO_3);

	return generalSBGC->_ParserCurrentStatus;
}
/**	@}
 */


/**	@addtogroup	Auto_PID
 * 	@{
 */
/**	@brief	Starts automatic PID calibration
 *
 *	@attention	When finished, the controller sends
 *				a full set of tuned parameters:\n
 *				See @ref Profile_Params_Ext,\n
 *				@ref Profile_Params_Ext_2 and\n
 *				@ref Profile_Params_Ext_3 modules\n
 *				To interrupt currently running auto-tuning process
 *				send this command with zero values in autoPID structure
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	*autoPID - structure with written auto-PID configurations
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_TuneAutoPID (GeneralSBGC_t *generalSBGC, const AutoPID_t *autoPID, ConfirmationState_t *confirmationState)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_AUTO_PID);
	WriteBuff(&cmd, autoPID, sizeof(AutoPID_t), PM_AUTO_PID);
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Starts automatic PID calibration ver.2
 *
 *	@attention	Firmware: 2.70+\n
 *				When finished, the controller sends
 *				a full set of tuned parameters:\n
 *				See @ref Profile_Params_Ext,\n
 *				@ref Profile_Params_Ext_2 and\n
 *				@ref Profile_Params_Ext_3 modules\n
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	*autoPID2 - structure with written auto-PID configurations
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_TuneAutoPID_2 (GeneralSBGC_t *generalSBGC, const AutoPID_2_t *autoPID2, ConfirmationState_t *confirmationState)
{
	if (generalSBGC->_firmwareVersion < 2700)
		return NOT_SUPPORTED_BY_FIRMWARE;

	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_AUTO_PID2);
	WriteBuff(&cmd, autoPID2, sizeof(AutoPID_2_t), PM_AUTO_PID_2);
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Parses progress of PID auto tuning data
 *
 *	@param	*cmd - SerialCommand structure, receiving
 *			progress of PID auto tuning realtime data
 *	@param	*autoPID_State - structure storing progress of PID
 */
void SBGC32_ParseAutoPID_StateCmd (SerialCommand_t *cmd, AutoPID_State_t *autoPID_State)
{
	cmd->readPos = 0;
	ReadBuff(cmd, autoPID_State, sizeof(AutoPID_State_t), PM_AUTO_PID_STATE);
}
/**	@}
 */


/**	@addtogroup	Motors_State
 * 	@{
 */
/**	@brief	Switches motors ON
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SetMotorsON (GeneralSBGC_t *generalSBGC, ConfirmationState_t *confirmationState)
{
	SBGC32_SendEmptyCommand(generalSBGC, CMD_MOTORS_ON);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, CMD_MOTORS_ON);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Switches motors OFF
 *
 *	@attention	Firmware: 2.68b7+ for parameter <mode>
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 * 	@param	mode - possible way for turn off motors
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SetMotorsOFF (GeneralSBGC_t *generalSBGC, MotorsMode_t mode, ConfirmationState_t *confirmationState)
{
	if (generalSBGC->_firmwareVersion < 2687)
		return NOT_SUPPORTED_BY_FIRMWARE;

	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_MOTORS_OFF);
	WriteByte(&cmd, mode);
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}
/**	@}
 */


/**	@addtogroup	Boot_Mode
 * 	@{
 */
/**	@brief	Enters bootloader mode to upload firmware
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SetBootMode (GeneralSBGC_t *generalSBGC)
{
	SBGC32_SendEmptyCommand(generalSBGC, CMD_BOOT_MODE_3);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Enters bootloader mode to upload firmware
 * 			in extended format
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *	@param	confirm - yes/no confirmation after reset
 *	@param	delayMs - waiting for a given time before reset
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SetBootModeExt (GeneralSBGC_t *generalSBGC, Boolean_t confirm, ui16 delayMs, ConfirmationState_t *confirmationState)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_BOOT_MODE_3);
	WriteByte(&cmd, confirm);
	WriteWord(&cmd, delayMs);
	SBGC32_TX(generalSBGC, &cmd);
	if (confirm == TRUE__)
		SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Resets device
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *	@param	flag - reset action
 *	@param	delayMs - waiting for a given time before reset
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_Reset (GeneralSBGC_t *generalSBGC, ResetFlag_t flag, ui16 delayMs)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_RESET);
	WriteByte(&cmd, flag);
	WriteWord(&cmd, delayMs);
	SBGC32_TX(generalSBGC, &cmd);
	/** Confirmation is processed manually */
	return generalSBGC->_ParserCurrentStatus;
}
/**	@}
 */


/**	@addtogroup	Scripts
 * 	@{
 */
/**	@brief	Start or stop user-written script
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *	@param	mode - script's action
 *	@param	slot - script's slot
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_RunScript (GeneralSBGC_t *generalSBGC, ScriptMode_t mode, ScriptSlotNum_t slot, ConfirmationState_t *confirmationState)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_RUN_SCRIPT);
	WriteByte(&cmd, mode);
	WriteByte(&cmd, slot);
	WriteEmptyBuff(&cmd, 32);  // reserved[32]
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Parse script debug information data
 *
 *	@param	*cmd - SerialCommand structure, receiving
 *			script debug information realtime data
 *	@param	*scriptDebugInfo - structure storing
 *			script debug information
 */
void SBGC32_ParseScriptDebugInfoCmd (SerialCommand_t *cmd, ScriptDebugInfo_t *scriptDebugInfo)
{
	cmd->readPos = 0;
	scriptDebugInfo->curComCounter = ReadWord(cmd);
	scriptDebugInfo->errCode = ReadByte(cmd);
}
/**	@}
 */


/**	@addtogroup	State_Vars
 * 	@{
 */
/**	@brief	Writes system persistent state variables, cumulative
 * 			statistics and maintenance data
 *
 * 	@attention	Firmware: 2.68b7+ (“Extended” family only)
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *	@param	*stateVars - structure with written var state parameters
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_WriteStateVars (GeneralSBGC_t *generalSBGC, const StateVars_t *stateVars, ConfirmationState_t *confirmationState)
{
	if (generalSBGC->_firmwareVersion < 2687)
		return NOT_SUPPORTED_BY_FIRMWARE;

	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_WRITE_STATE_VARS);
	WriteBuff(&cmd, stateVars, sizeof(StateVars_t), PM_STATE_VARS);
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Request reading system persistent state variables, cumulative
 * 			statistics and maintenance data
 *
 * 	@attention	Firmware: 2.68b7+ (“Extended” family only)
 *
 * 	@param 	*generalSBGC - serial connection descriptor
 *	@param	*stateVars - structure for storing var state parameters
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_ReadStateVars (GeneralSBGC_t *generalSBGC, StateVars_t *stateVars)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_READ_STATE_VARS);
	SBGC32_TX_RX(generalSBGC, &cmd, CMD_READ_STATE_VARS);
	ReadBuff(&cmd, stateVars, sizeof(StateVars_t), PM_STATE_VARS);
	return generalSBGC->_ParserCurrentStatus;
}
/**	@}
 */


/**	@addtogroup	Service_Other
 * 	@{
 */
/**	@brief	Parse event message
 *
 *	@attention	Firmware: 2.65+
 *
 *	@param	*cmd - SerialCommand structure,
*			receiving event data
 *	@param	*event - structure storing event data
 */
void SBGC32_ParseEventCmd (SerialCommand_t *cmd, Event_t *event)
{
	if (cmd->readPos != 0)
		cmd->readPos = 0;

	ReadBuff(cmd, event, sizeof(Event_t), PM_DEFAULT_8BIT);
}


/**	@brief	Trigger output pin
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	pinID - trigger pin identifier
 *	@param	state - the physical state of the pin
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SetTriggerPin (GeneralSBGC_t *generalSBGC, TriggerPinID_t pinID, PinState_t state, ConfirmationState_t *confirmationState)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_TRIGGER_PIN);
	WriteByte(&cmd, pinID);
	WriteByte(&cmd, state);
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	/* Pin doesn't must be occupied for other functions and was really triggered */
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Execute menu command
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	cmdID - menu command identifier
 *	@param	*confirmationState - confirmation structure
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_ExecuteMenu (GeneralSBGC_t *generalSBGC, MenuCommands_t cmdID, ConfirmationState_t *confirmationState)
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_EXECUTE_MENU);
	WriteByte(&cmd, cmdID);
	SBGC32_TX(generalSBGC, &cmd);
	SBGC32_CheckConfirmation(generalSBGC, confirmationState, cmd.commandID);
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Set output PWM signal on the specified pins
 *
 *	@attention	Although command takes 8 values, the real
 *				number of hardware outputs depends on
 *				board version and may be less
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	*servoTime - array with PWM duty cycle
 *			values for each output
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SetServoOut (GeneralSBGC_t *generalSBGC, const i16 servoTime [8])
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_SERVO_OUT);
	FOR_(i, 8) WriteWord(&cmd, servoTime[i]);
	SBGC32_TX(generalSBGC, &cmd);
	/* no need confirmation */
	return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Play melody by motors or emit standard beep sound
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	*beeperSettings - structure with prescribed
 *			beeper playback settings
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_PlayBeeper (GeneralSBGC_t *generalSBGC, const BeeperSettings_t *beeperSettings)
{
    SerialCommand_t cmd;
    InitCmdWrite(&cmd, CMD_BEEP_SOUND);
    WriteWord(&cmd, beeperSettings->mode);
    WriteByte(&cmd, beeperSettings->noteLength);
    WriteByte(&cmd, beeperSettings->decayFactor);
	WriteEmptyBuff(&cmd, 8);  // reserved[8]

    if (beeperSettings->mode == BM_BEEPER_MODE_CUSTOM_MELODY)
    	FOR_(i, beeperSettings->notesQuan) WriteWord(&cmd, beeperSettings->notesFreqHz[i]);

    SBGC32_TX(generalSBGC, &cmd);
    /* no need confirmation */
    return generalSBGC->_ParserCurrentStatus;
}


/**	@brief	Signs the user's message
 *
 *	@param	*generalSBGC - serial connection descriptor
 *	@param	signType - set of keys to be used
 *	@param	*txMessage - the user's input message
 *	@param	*rxMessage - signed message
 *
 *	@return Communication status
 */
TxRxStatus_t SBGC32_SignMessage (GeneralSBGC_t *generalSBGC, ui8 signType, const char txMessage [MAX_MESSAGE_LENGTH], char rxMessage [MAX_MESSAGE_LENGTH])
{
	SerialCommand_t cmd;
	InitCmdWrite(&cmd, CMD_SIGN_MESSAGE);
	WriteByte(&cmd, signType);
	WriteBuff(&cmd, txMessage, MAX_MESSAGE_LENGTH, PM_DEFAULT_8BIT);

	if (CheckReceipt(generalSBGC, SBGC32_TX_RX(generalSBGC, &cmd, CMD_SIGN_MESSAGE), "Sign Message:") == TX_RX_OK)
		ReadBuff(&cmd, rxMessage, MAX_MESSAGE_LENGTH, PM_DEFAULT_8BIT);

	return generalSBGC->_ParserCurrentStatus;
}
/**	@}
 */

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
/*					https://www.basecamelectronics.com  			  */
/* __________________________________________________________________ */
