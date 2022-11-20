/** ____________________________________________________________________
 *
 * 	@file		gimbalControl.h
 *
 *	@brief 		Gimbal control header file
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
 * 	@defgroup	Gimbal_Control SBGC32 Gimbal Control
 *	@ingroup	Sources
 *		@brief	Gimbal Control Title Module
 *  ____________________________________________________________________
 *
 * 	@defgroup	Control Control
 * 	@ingroup	Gimbal_Control
 * 		@brief	Control Module
 *
 *				Covered Commands:
 *
 *				### CMD_CONTROL
 *
 *  @defgroup	Control_Config Control Configuration
 *  @ingroup	Gimbal_Control
 *  	@brief	Control Configuration Module
 *
 *  			Covered Commands:
 *
 *				### CMD_CONTROL_CONFIG
 *
 *  @defgroup	Virt_Channels Virtual Channels
 *  @ingroup	Gimbal_Control
 *  	@brief	Virtual Channels Module
 *
 *  			Covered Commands:
 *
 *				### CMD_API_VIRT_CH_CONTROL
 *				### CMD_API_VIRT_CH_HIGH_RES
 *	____________________________________________________________________
 */

#ifndef 	_GIMBAL_CONTROL_H_
#define 	_GIMBAL_CONTROL_H_

#ifdef 		__cplusplus
extern 		"C" {
#endif
/*  = = = = = = = = = = = = = = = = = = = = = = = */

#include 	"../core/core.h"


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *								  Macros & Constants
 */
#define	 	ANGLE_FULL_TURN 		16384

/*	Conversion from degree/sec to units
	that command understand */
#define		SPEED_SCALE  			(1.0f / 0.1220740379f)
#define		SPEED_TO_VALUE(val)		((val) * SPEED_SCALE)

#define 	DEGREE_ANGLE_SCALE 		((float)ANGLE_FULL_TURN / 360.0f)
#define 	ANGLE_DEGREE_SCALE 		(360.0f / (float)ANGLE_FULL_TURN)

/* Conversions for angle in degrees to angle
	in SBGC 14 bit representation, and back */
#define 	DEGREE_TO_ANGLE(val) 	((val) * DEGREE_ANGLE_SCALE)
#define 	ANGLE_TO_DEGREE(val) 	((val) * ANGLE_DEGREE_SCALE)

/*	Same thing, optimized for integers */
#define 	DEGREE_TO_ANGLE_INT(val)\
									((i32)(val) * ANGLE_FULL_TURN / 360)
#define		DEGREE_01_TO_ANGLE_INT(val)\
									((i32)(val) * ANGLE_FULL_TURN / 3600)
#define 	ANGLE_TO_DEGREE_INT(val)\
									((i32)(val) * 360 / ANGLE_FULL_TURN)
#define 	ANGLE_TO_DEGREE_01_INT(val)\
									((i32)(val) * 3600 / ANGLE_FULL_TURN)


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *									   Control Flags
 */
/**	@addtogroup	Control
 *	@{
 */
/**	@note	Control_t.controlMode
 *
 *	Can be combined with @ref ControlFlags_t
 */
typedef enum
{
	CtrlM_MODE_NO_CONTROL			= 0,
	CtrlM_MODE_SPEED				= 1,
	CtrlM_MODE_ANGLE				= 2,
	CtrlM_MODE_SPEED_ANGLE			= 3,
	CtrlM_MODE_RC  					= 4,
	CtrlM_MODE_ANGLE_REL_FRAME		= 5,
	/** frw. ver. 2.66b2+ */
	CtrlM_MODE_RC_HIGH_RES			= 6

}	ControlMode_t;

/**	@note	Control_t.controlMode
 *
 *	Can be combined with @ref ControlMode_t
 */
typedef enum
{
	/** Applicable for CtrlM_MODE_ANGLE or CtrlM_MODE_ANGLE_REL_FRAME
	   frw. ver. 2.70b1+ */
	CtrlF_CONTROL_FLAG_TARGET_PRECISE
									= BIT_5_SET,
	/** Applicable for CtrlM_MODE_ANGLE or CtrlM_MODE_ANGLE_REL_FRAME
	   frw. ver. 2.62b7+ */
	CtrlF_CONTROL_FLAG_AUTO_TASK	= BIT_6_SET,

	/** Applicable for CtrlM_MODE_RC
	   frw. ver. 2.62b7+ */
	CtrlF_CONTROL_FLAG_FORCE_RC_SPEED
									= BIT_6_SET,
	/** frw. ver. 2.60+ */
	CtrlF_CONTROL_FLAG_HIGH_RES_SPEED
									= BIT_7_SET

}	ControlFlags_t;

/** @brief	Used as a mask to extract the
 * 			Control mode value from
 * 			Control_t.controlMode
 *
 * 	@param 	value (Control_t.controlMode)
 *
 * 	@return	Control mode
 */
static inline ControlMode_t GetControlMode (ui8 value)
{
	return (ControlMode_t)(value & 0b00001111);
}

/** @brief	Used as a mask to extract the
 * 			Control flag value from
 * 			Control_t.controlMode
 *
 * 	@param 	value (Control_t.controlMode)
 *
 * 	@return	Control flag
 */
static inline ControlFlags_t GetControlFlag (ui8 value)
{
	return (ControlFlags_t)(value & 0b11110000);
}
/**	@}
 */


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *								Control Config Flags
 */
/**	@addtogroup	Control_Config
 *	@{
 */
/** @note	ControlConfig_t.flags
 */
typedef enum
{
	RTCCF_CONTROL_CONFIG_FLAG_CONFIRM
									= 0,
	RTCCF_CONTROL_CONFIG_FLAG_NO_CONFIRM
									= BIT_0_SET

}	RealTimeControlConfigFlags_t;
/**	@}
 */


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 * 						  Realtime Control Structure
 */
/**	@addtogroup	Control
 * 	@{
 */
/**	@brief	Part of Control_t structure
 *
 * 	@note	Control_t.AxisC
 */
typedef struct __PACKED__
{
	i16     speed,                              	/*!<  Units: 0.1220740379 deg/sec or
													  	  Units: 0.001 deg/sec, if CtrlF_CONTROL_FLAG_HIGH_RES_SPEED is set				*/
			angle;                              	/*!<  Units: 0.02197265625 degree													*/

}           AxisC_t;

/** @brief	Structure type for real-time gimbal
 * 			position control operation
 *
 *	@note	<b><i>Parameters:</i></b>
 *
 *			CMD_CONTROL
 *
 *			TX 15 bytes
 *
 *	@ref	SBGC32_Control function
 */
typedef struct __PACKED__
{
	/* frw. ver. 2.55b5+ */
    ui8     controlMode [3];						/*!<  See @ref ControlMode_t and @ref ControlFlags_t enumerations					*/

    AxisC_t	AxisC [3];								/*!<  ROLL : PITCH : YAW															*/

}           Control_t;
/**	@}
 */


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 * 				   Realtime Control Config Structure
 */
/**	@addtogroup	Control_Config
 * 	@{
 */
/**	@brief	Part of ControlConfig_t structure
 *
 * 	@note	ControlConfig_t.AxisCC
 */
typedef struct __PACKED__
{
	ui8     angleLPF;                           	/*!<  0 --> 15																		*/
	ui8     speedLPF;                           	/*!<  0 --> 15																		*/
	ui8     RC_LPF;                             	/*!<  0 --> 15 (255*). Range depends on the flag “Extend LPF range” in GUI settings */
	ui16    ACC_Limit;                          	/*!<  0 --> 65535. Units: degrees/sec²												*/
	ui8     reserved1 [2];

}           AxisCC_t;

/** @brief	Structure type for real-time gimbal
 * 			position control configuration
 *
 *	@note	<b><i>Parameters:</i></b>
 *
 *			CMD_CONTROL_CONFIG
 *
 *			TX 41 bytes
 *
 *	@ref	SBGC32_ControlConfig function
 */
typedef struct __PACKED__
{
    ui16    timeoutMS;                              /*!<  0 --> 65535. Units: ms														*/

    ui8     ch1_Priority,                           /*!<  0 - do not change the priority...												*/
            ch2_Priority,                           /*!<  ...1 --> 255 - set the priority of a given channel...							*/
            ch3_Priority,                           /*!<  ...In case of concurrent CMD_CONTROL commands, they will be accepted...		*/
            ch4_Priority,                           /*!<  ...only on a channel that has higher or equal priority than others...			*/
            thisChPriority;                         /*!<  ...Default value is 0 for all channels after startup							*/

    AxisCC_t	AxisCC [3];							/*!<  ROLL : PITCH : YAW															*/

    ui8     RC_ExpoRate;                            /*!<  0 --> 100. Default value is taken from the "RC_EXPO_RATE" GUI parameter		*/
    ui16    flags;                                  /*!<  See @ref RealTimeControlConfigFlags_t enumeration								*/
    ui8     reserved2 [10];

}           ControlConfig_t;
/**	@}
 */


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 * 								 Function Prototypes
 */
/**	@addtogroup	Control
 * 	@{
 */
TxRxStatus_t SBGC32_Control	(GeneralSBGC_t *generalSBGC, const Control_t *control);
/**	@}
 */


/**	@addtogroup	Control_Config
 * 	@{
 */
TxRxStatus_t SBGC32_ControlConfig (GeneralSBGC_t *generalSBGC, const ControlConfig_t *controlConfig, ConfirmationState_t *confirmationState);
/**	@}
 */


/**	@addtogroup	Virt_Channels
 * 	@{
 */
TxRxStatus_t SBGC32_SetAPI_VirtChControl (GeneralSBGC_t *generalSBGC, const i16 API_VirtCh [VIRT_CHANNELS_QUANTITY]);
TxRxStatus_t SBGC32_SetAPI_VirtChHR_Control (GeneralSBGC_t *generalSBGC, const i16 *API_VirtCh, ui8 chQuan);
/**	@}
 */

/*  = = = = = = = = = = = = = = = = = = = = = = = */
#ifdef 		__cplusplus
			}
#endif

#endif		/* _GIMBAL_CONTROL_H_ */
