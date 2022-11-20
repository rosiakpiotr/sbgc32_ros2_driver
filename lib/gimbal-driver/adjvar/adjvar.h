/** ____________________________________________________________________
 *
 * 	@file		adjvar.h
 *
 *	@brief 		Adjustable variables header file
 *
 *				<center><a href="https://www.basecamelectronics.com">
 *				www.basecamelectronics.com</a>
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
/** ____________________________________________________________________
 *
 * 	@defgroup	Adjvar SBGC32 Adjustable Variables
 *	@ingroup	Sources
 *		@brief	SBGC32 Adjustable Variables Title Module
 *  ____________________________________________________________________
 *
 * 	@defgroup	Adjvar_Values Adjustable Variable Values
 * 	@ingroup	Adjvar
 * 		@brief	Adjustable Variable Values Module
 *
 * 				Covered Commands:
 *
 *				### CMD_SET_ADJ_VARS_VAL
 *				### CMD_GET_ADJ_VARS_VAL
 *				### CMD_SAVE_PARAMS_3
 *
 * 	@defgroup	Adjvar_Cfg Adjustable Variable Configurations
 * 	@ingroup	Adjvar
 * 		@brief	Adjustable Variable Configurations Module
 *
 * 				Covered Commands:
 *
 *				### CMD_WRITE_ADJ_VARS_CFG
 *				### CMD_READ_ADJ_VARS_CFG
 *
 * 	@defgroup	Adjvar_State Adjustable Variable States
 * 	@ingroup	Adjvar
 * 		@brief	Adjustable Variable States Module
 *
 * 				Covered Commands:
 *
 *				### CMD_ADJ_VARS_STATE
 *	____________________________________________________________________
 */

#ifndef     _ADJVAR_H_
#define     _ADJVAR_H_

#ifdef 		__cplusplus
extern 		"C" {
#endif
/*  = = = = = = = = = = = = = = = = = = = = = = = */

#include 	"../core/core.h"


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *										   Constants
 */
/**	@addtogroup	Adjvar
 * 	@{
 */
#define 	ADJ_VARS_QANTITY		66				/*!<  Number of adjustable variables for the latest firmware version				*/
#define		ADJ_VAR_MAX_NAME_LENGTH	30				/*!<  Maximal name length of adjustable variable for debug information				*/


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *				Adjustable Variables General Objects
 */
/**	@brief	Adjustable variables general enumeration
 *
 *	@note	AdjVarsCfg_t.AnalogSlot
 * 	@note	AdjVarsState_t.analogVarID
 * 	@note	AdjVarsState_t.lutVarID
 */
typedef enum
{
	ADJ_VAR_P_ROLL 					= 0,
	ADJ_VAR_P_PITCH,
	ADJ_VAR_P_YAW,
	ADJ_VAR_I_ROLL,
	ADJ_VAR_I_PITCH,
	ADJ_VAR_I_YAW,
	ADJ_VAR_D_ROLL,
	ADJ_VAR_D_PITCH,
	ADJ_VAR_D_YAW,
	ADJ_VAR_POWER_ROLL,
	ADJ_VAR_POWER_PITCH,
	ADJ_VAR_POWER_YAW,
	ADJ_VAR_ACC_LIMITER,
	ADJ_VAR_FOLLOW_SPEED_ROLL,
	ADJ_VAR_FOLLOW_SPEED_PITCH,
	ADJ_VAR_FOLLOW_SPEED_YAW,
	ADJ_VAR_FOLLOW_LPF_ROLL,
	ADJ_VAR_FOLLOW_LPF_PITCH,
	ADJ_VAR_FOLLOW_LPF_YAW,
	ADJ_VAR_RC_SPEED_ROLL,
	ADJ_VAR_RC_SPEED_PITCH,
	ADJ_VAR_RC_SPEED_YAW,
	ADJ_VAR_RC_LPF_ROLL,
	ADJ_VAR_RC_LPF_PITCH,
	ADJ_VAR_RC_LPF_YAW,
	ADJ_VAR_RC_TRIM_ROLL,
	ADJ_VAR_RC_TRIM_PITCH,
	ADJ_VAR_RC_TRIM_YAW,
	ADJ_VAR_RC_DEADBAND,
	ADJ_VAR_RC_EXPO_RATE,
	ADJ_VAR_FOLLOW_PITCH,
	ADJ_VAR_RC_FOLLOW_YAW_PITCH,
	ADJ_VAR_FOLLOW_DEADBAND,
	ADJ_VAR_FOLLOW_EXPO_RATE,
	ADJ_VAR_FOLLOW_ROLL_MIX_START,
	ADJ_VAR_FOLLOW_ROLL_MIX_RANGE,
	ADJ_VAR_GYRO_TRUST,
	ADJ_VAR_FRAME_HEADING,
	ADJ_VAR_GYRO_HEADING_CORR,
	ADJ_VAL_ACC_LIMITER_ROLL,
	ADJ_VAL_ACC_LIMITER_PITCH,
	ADJ_VAL_ACC_LIMITER_YAW,
	ADJ_VAR_PID_GAIN_ROLL,
	ADJ_VAR_PID_GAIN_PITCH,
	ADJ_VAR_PID_GAIN_YAW,
	ADJ_VAR_LPF_FREQ_ROLL,
	ADJ_VAR_LPF_FREQ_PITCH,
	ADJ_VAR_LPF_FREQ_YAW,
	ADJ_VAR_TIMELAPSE_TIME,
	ADJ_VAR_MAV_CTRL_MODE,
	ADJ_VAR_H_CORR_FACTOR,
	ADJ_VAR_SW_LIM_MIN_ROLL,
	ADJ_VAR_SW_LIM_MAX_ROLL,
	ADJ_VAR_SW_LIM_MIN_PITCH,
	ADJ_VAR_SW_LIM_MAX_PITCH,
	ADJ_VAR_SW_LIM_MIN_YAW,
	ADJ_VAR_SW_LIM_MAX_YAW,
	ADJ_VAR_FOLLOW_RANGE_ROLL,
	ADJ_VAR_FOLLOW_RANGE_PITCH,
	ADJ_VAR_FOLLOW_RANGE_YAW,
	ADJ_VAR_AUTO_PID_TARGET,
	ADJ_VAR_RC_MODE_ROLL,
	ADJ_VAR_RC_MODE_PITCH,
	ADJ_VAR_RC_MODE_YAW,
	ADJ_VAR_EULER_ORDER,
	ADJ_VAR_FOLLOW_IN_DBAND

}	AdjVarsList_t;


/**	@note	AdjVarsGeneral_t.changeFlag
 */
typedef enum
{
	NOT_CHANGED						= 0,
	CHANGED

}	VarChangeFlag_t;


/**	@note	AdjVarsGeneral_t.saveFlag
 */
typedef enum
{
	SAVED							= 0,
	NOT_SAVED

}	VarSaveFlag_t;


/**	@brief	Adjustable variables general structure
 */
typedef struct
{
	AdjVarsList_t	ID;								/*!<  Adjustable variable ID  														*/

	#ifdef	SBGC_DEBUG_MODE

		char		name [ADJ_VAR_MAX_NAME_LENGTH];	/*!<  Adjustable variable name														*/

	#endif

	i16				minValue,						/*!<  Adjustable variable minimal value												*/
					maxValue;						/*!<  Adjustable variable maximal value												*/

	VarTypes_t	 	varType;						/*!<  Type of adjustable variable  													*/

	i32		 		value;							/*!<  Adjustable variable value  													*/

	VarChangeFlag_t changeFlag;						/*!<  Service flag set when changing the value of a adjustable variable				*/
	VarSaveFlag_t	saveFlag;						/*!<  Service flag set when saving the changed value of the adjustable variable		*/

}					AdjVarsGeneral_t;


#ifdef	SBGC_DEBUG_MODE

	/**	@brief	Adjustable variables
	 * 			debug info structure
	 */
	typedef struct
	{
		const AdjVarsList_t	ID;						/*!<  Adjustable variable ID  														*/

		char		name [ADJ_VAR_MAX_NAME_LENGTH];	/*!<  Adjustable variable name														*/

		const i32	minValue,						/*!<  Adjustable variable minimal value												*/
					maxValue;						/*!<  Adjustable variable maximal value												*/

		const VarTypes_t	varType;				/*!<  Type of adjustable variable  													*/

	}				AdjVarsDebugInfo_t;
	/**	@}
	 */

#endif
/**	@}
 */


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *		Adjustable Variable Configuration Structures
 */
/**	@addtogroup	Adjvar_Cfg
 * 	@{
 */
/**	@brief	Part of AdjVarsCfg_t structure
 *
 * 	@note	AdjVarsCfg_t.TriggerSlot
 */
typedef struct __PACKED__
{
	ui8		triggerSrcCh,							/*!<  See @ref RC_MapSourceType_t and @ref RC_MapSource_t enumerations				*/
			triggerAction [5];						/*!<  See @ref MenuCommands_t enumeration											*/

}			TriggerSlot_t;

/**	@brief	Part of AdjVarsCfg_t structure
 *
 * 	@note	AdjVarsCfg_t.AnalogSlot
 */
typedef struct __PACKED__
{
	ui8		analogSrc,								/*!<  See @ref RC_MapSourceType_t and @ref RC_MapSource_t enumerations				*/
			varID,									/*!<  See @ref AdjVarsList_t enumeration											*/
			minVal,									/*!<  Is specify a working range, that is mapped...									*/
			maxVal;									/*!<  ...to a native range of particular parameter									*/

}			AnalogSlot_t;

/**	@brief	Structure type for work with
 *  		AdjVarsCfg parameters
 *
 *	@note	<b><i>Parameters:</i></b>
 *
 *			CMD_WRITE_ADJ_VARS_CFG
 *			CMD_READ_ADJ_VARS_CFG
 *
 *			TX/RX 128 bytes
 *
 *	@ref	SBGC32_WriteAdjVarsCfg function\n
 *	@ref	SBGC32_ReadAdjVarsCfg function
 */
typedef struct __PACKED__
{
	TriggerSlot_t	TriggerSlot [10];

	AnalogSlot_t	AnalogSlot [15];

	ui8		reserved [8];

}			AdjVarsCfg_t;
/**	@}
 */


/**	@addtogroup	Adjvar_State
 * 	@{
 */
/**	@brief	Structure type for work with
 *  		AdjVarsState parameters
 *
 *	@note	<b><i>Parameters:</i></b>
 *
 *			Firmware: 2.62b5+, beginning with
 *			<triggerSlot> parameter
 *
 *			CMD_ADJ_VARS_STATE
 *
 *			TX/RX 7/15 bytes (old ver. 2/15 bytes)
 *
 *			Request the state of
 *			adjustable variables
 *
 *	@ref	SBGC32_RequestAdjVarsState function
 */
typedef struct __PACKED__
{
	/* Firmware ver. prior to 2.62b5 */
	/* Sent data */
	ui8		triggerSlot__old;						/*!<  0 --> 9  																		*/
	ui8		analogSlot;								/*!<  0 --> 14  																	*/
	/* Received data */
	i16 	triggerRC_Data__old;					/*!<  -500 --> 500. RC signal for the "trigger" variable slot						*/
	ui8		triggerAction__old;						/*!<  See @ref MenuCommands_t enumeration											*/
	i16		analogRC_Data;							/*!<  -500 --> 500. RC signal for the "analog" variable slot						*/
	i32		analogValue;							/*!<  Current value of the variable after all calculations							*/
	ui8		reserved [6];

	/* Firmware ver. 2.62b5+ */
	/* Sent data */
	ui8		triggerSlot;							/*!<  0 --> 9  																		*/
	ui16	analogSrcID;							/*!<  "Trigger" slot number to show its state										*/
	ui8		analogVarID;							/*!<  See @ref AdjVarsList_t enumeration. Variable ID to show its value				*/
	ui16	lutSrcID;								/*!<  Signal source to show its value												*/
	ui8		lutVarID;								/*!<  See @ref AdjVarsList_t enumeration. Variable ID to show its value				*/
	/* Received data */
	i16 	triggerRC_Data;							/*!<  -16384 --> 16384. RC signal for the "trigger" variable slot					*/
	ui8		triggerAction;							/*!<  See @ref MenuCommands_t enumeration											*/
	i16		analogSrcValue;							/*!<  -16384 --> 16384. Signal value requested in the analogSrcID					*/
	float 	analogVarValue;							/*!<  Value of variable requested in the analogVarID								*/
	i16		lutSrcValue;							/*!<  -16384 --> 16384. Signal value requested in the lutSrcID 						*/
	float	lutVarValue;							/*!<  Current value of variable requested in the lutVarID							*/

}			AdjVarsState_t;
/**	@}
 */


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 * 								 Function Prototypes
 */
/**	@addtogroup	Adjvar_Values
 * 	@{
 */
void InitAdjVar (AdjVarsGeneral_t *adjVarsGeneral, const AdjVarsDebugInfo_t *adjVarReference);
void EditAdjVarValue (AdjVarsGeneral_t *adjVarsGeneral, i32 value);
TxRxStatus_t SBGC32_SetAdjVarValues (GeneralSBGC_t *generalSBGC, AdjVarsGeneral_t *adjVarsGeneral, ui8 adjVarQuan, ConfirmationState_t *confirmationState);
TxRxStatus_t SBGC32_GetAdjVarValue (GeneralSBGC_t *generalSBGC, AdjVarsGeneral_t *adjVarsGeneral);
TxRxStatus_t SBGC32_GetAdjVarValues (GeneralSBGC_t *generalSBGC, AdjVarsGeneral_t *adjVarsGeneral, ui8 adjVarQuan);
TxRxStatus_t SBGC32_SaveAdjVarToEEPROM (GeneralSBGC_t *generalSBGC, AdjVarsGeneral_t *adjVarsGeneral, ConfirmationState_t *confirmationState);
TxRxStatus_t SBGC32_SaveAdjVarsToEEPROM (GeneralSBGC_t *generalSBGC, AdjVarsGeneral_t *adjVarsGeneral, ui8 adjVarQuan, ConfirmationState_t *confirmationState);
TxRxStatus_t SBGC32_SaveAllActiveAdjVarsToEEPROM (GeneralSBGC_t *generalSBGC, ConfirmationState_t *confirmationState);
/**	@}
 */


/**	@addtogroup	Adjvar_Cfg
 * 	@{
 */
TxRxStatus_t SBGC32_WriteAdjVarsCfg (GeneralSBGC_t *generalSBGC, const AdjVarsCfg_t *adjVarsCfg, ConfirmationState_t *confirmationState);
TxRxStatus_t SBGC32_ReadAdjVarsCfg (GeneralSBGC_t *generalSBGC, AdjVarsCfg_t *adjVarsCfg);
/**	@}
 */


/**	@addtogroup	Adjvar_State
 * 	@{
 */
TxRxStatus_t SBGC32_RequestAdjVarsState (GeneralSBGC_t *generalSBGC, AdjVarsState_t *adjVarsState);
/**	@}
 */

/*  = = = = = = = = = = = = = = = = = = = = = = = */
#ifdef 		__cplusplus
			}
#endif

#endif      /* _ADJVAR_H_ */
