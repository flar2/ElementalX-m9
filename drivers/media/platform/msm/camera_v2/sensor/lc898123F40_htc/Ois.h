/**
 * @brief		OIS system common header for LC898123 F40
 * 				Defines, Structures, function prototypes
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 *
 * @file		Ois.h
 * @date		svn:$Date:: 2016-06-22 10:57:58 +0900#$
 * @version	svn:$Revision: 59 $
 * @attention
 **/
#ifndef OIS_H_
#define OIS_H_

typedef	char				 INT8;
typedef	short				 INT16;
typedef	long                 INT32;
typedef	long long            INT64;
typedef	unsigned char       UINT8;
typedef	unsigned short      UINT16;
typedef	unsigned long       UINT32;
typedef	unsigned long long	UINT64;

//#define		_BIG_ENDIAN_
/*HTC_START*/
// FRA is used for module vendor, so currently remove it
#if 0
#define		USE_FRA			// uncomment if use FRA function
#else
//#define		USE_FRA			// uncomment if use FRA function
#endif
/*HTC_END*/

#include	"OisAPI.h"
#include	"OisLc898123F40.h"

/**************** Model name *****************/
/**************** FW version *****************/
 #define	MDL_VER			0x05
 #define	FW_VER			0x06

/**************** Select Mode **************/
#define		NEUTRAL_CENTER				//!< Upper Position Current 0mA Measurement
#define		NEUTRAL_CENTER_FINE			//!< Optimize natural center current
//#define		SEL_CLOSED_AF

/**************** Filter sampling **************/
#define	FS_FREQ			20019.53125F

#define	GYRO_SENSITIVITY	87.5		//!< Gyro sensitivity LSB/dps

// Command Status
#define		EXE_END		0x00000002L		//!< Execute End (Adjust OK)
#define		EXE_HXADJ	0x00000006L		//!< Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0000000AL		//!< Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x00000012L		//!< Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x00000022L		//!< Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x00000042L		//!< Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x00000082L		//!< Adjust NG : Y Gyro NG (offset)
#define		EXE_ERR		0x00000099L		//!< Execute Error End
#ifdef	SEL_CLOSED_AF
 #define	EXE_HZADJ	0x00100002L		//!< Adjust NG : AF Hall NG (Gain or Offset)
 #define	EXE_LZADJ	0x00200002L		//!< Adjust NG : AF Loop NG (Gain)
#endif


// Common Define
#define	SUCCESS			0x00			//!< Success
#define	FAILURE			0x01			//!< Failure

#ifndef ON
 #define	ON				0x01		//!< ON
 #define	OFF				0x00		//!< OFF
#endif
 #define	SPC				0x02		//!< Special Mode

#define	X_DIR			0x00			//!< X Direction
#define	Y_DIR			0x01			//!< Y Direction
#define	Z_DIR			0x02			//!< Z Direction(AF)

#define	WPB_OFF			0x01			//!< Write protect OFF
#define WPB_ON			0x00			//!< Write protect ON

#define		SXGAIN_LOP		0x30000000
#define		SYGAIN_LOP		0x30000000
#define		XY_BIAS			0x40000000
#define		XY_OFST			0x00000303	//!< 7‚ÍHiZ
#ifdef	SEL_CLOSED_AF
#define		SZGAIN_LOP		0x30000000
#define		Z_BIAS			0x40000000
#define		Z_OFST			0x00030000	//!< 7‚ÍHiZ
#endif

struct STFILREG {						//!< Register data table
	UINT16	UsRegAdd ;
	UINT8	UcRegDat ;
} ;

struct STFILRAM {						//!< Filter coefficient table
	UINT16	UsRamAdd ;
	UINT32	UlRamDat ;
} ;

struct STCMDTBL {						//!< Command table
	UINT16 Cmd ;
	UINT32 UiCmdStf ;
	void ( *UcCmdPtr )( void ) ;
} ;

/************************************************/
/*	Command										*/
/************************************************/
#define		CMD_IO_ADR_ACCESS				0xC000				//!< IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				//!< IO Read Access
#define		CMD_RETURN_TO_CENTER			0xF010				//!< Center Servo ON/OFF choose axis
	#define		BOTH_SRV_OFF					0x00000000			//!< Both   Servo OFF
	#define		XAXS_SRV_ON						0x00000001			//!< X axis Servo ON
	#define		YAXS_SRV_ON						0x00000002			//!< Y axis Servo ON
	#define		BOTH_SRV_ON						0x00000003			//!< Both   Servo ON
	#define		ZAXS_SRV_OFF					0x00000004			//!< Z axis Servo OFF
	#define		ZAXS_SRV_ON						0x00000005			//!< Z axis Servo ON
#define		CMD_PAN_TILT					0xF011				//!< Pan Tilt Enable/Disable
	#define		PAN_TILT_OFF					0x00000000			//!< Pan/Tilt OFF
	#define		PAN_TILT_ON						0x00000001			//!< Pan/Tilt ON
#define		CMD_OIS_ENABLE					0xF012				//!< Ois Enable/Disable
	#define		OIS_DISABLE						0x00000000			//!< OIS Disable
	#define		OIS_ENABLE						0x00000001			//!< OIS Enable
	#define		OIS_ENA_NCL						0x00000002			//!< OIS Enable ( none Delay clear )
	#define		OIS_ENA_DOF						0x00000004			//!< OIS Enable ( Drift offset exec )
#define		CMD_MOVE_STILL_MODE				0xF013				//!< Select mode
	#define		MOVIE_MODE						0x00000000			//!< Movie mode
	#define		STILL_MODE						0x00000001			//!< Still mode
#define		CMD_CALIBRATION					0xF014				//!< Gyro offset re-calibration
#define		CMD_CHASE_CONFIRMATION			0xF015				//!< Hall Chase confirmation
#define		CMD_GYRO_SIG_CONFIRMATION		0xF016				//!< Gyro Signal confirmation
	// Calibration flags
	#define		HALL_CALB_FLG					0x00008000			//!< Hall calibration done bit
	#define		HALL_CALB_BIT					0x00FF00FF			//!< Caribration bit mask
	#define		GYRO_GAIN_FLG					0x00004000			//!< Gyro gain adjustment done bit
	#define		ANGL_CORR_FLG					0x00002000			//!< Angle correction adjustment done bit
	#define		OPAF_FST_FLG					0x00001000			//!< OPAF FST calibration done bit
	#define		CLAF_CALB_FLG					0x00000800			//!< CLAF Hall calibration done bit
	#define		HLLN_CALB_FLG					0x00000400			//!< Hall linear calibration done bit
	#define		CROS_TALK_FLG					0x00000200			//!< Cross talk calibration
#define		CMD_READ_STATUS					0xF100				//!< Status Read

#define		READ_STATUS_INI					0x01000000

#define		STBOSCPLL						0x00D00074			//!< STB OSC
	#define		OSC_STB							0x00000002			//!< OSC standby

// Calibration.h *******************************************************************
#define	HLXBO				0x00000001			//!< D/A Converter Channel Select OIS X BIAS
#define	HLYBO				0x00000002			//!< D/A Converter Channel Select OIS Y BIAS
#define	HLAFBO				0x00000004			//!< D/A Converter Channel Select AF BIAS
#define	HLAFO				0x00000008			//!< D/A Converter Channel Select AF OFFSET

// MeasureFilter.h *******************************************************************
typedef struct {
	INT32				SiSampleNum ;			//!< Measure Sample Number
	INT32				SiSampleMax ;			//!< Measure Sample Number Max

	struct {
		INT32			SiMax1 ;				//!< Max Measure Result
		INT32			SiMin1 ;				//!< Min Measure Result
		UINT32	UiAmp1 ;						//!< Amplitude Measure Result
		INT64		LLiIntegral1 ;				//!< Integration Measure Result
		INT64		LLiAbsInteg1 ;				//!< Absolute Integration Measure Result
		INT32			PiMeasureRam1 ;			//!< Measure Delay RAM Address
	} MeasureFilterA ;

	struct {
		INT32			SiMax2 ;				//!< Max Measure Result
		INT32			SiMin2 ;				//!< Min Measure Result
		UINT32	UiAmp2 ;						//!< Amplitude Measure Result
		INT64		LLiIntegral2 ;				//!< Integration Measure Result
		INT64		LLiAbsInteg2 ;				//!< Absolute Integration Measure Result
		INT32			PiMeasureRam2 ;			//!< Measure Delay RAM Address
	} MeasureFilterB ;
} MeasureFunction_Type ;


/*** caution [little-endian] ***/

#ifdef _BIG_ENDIAN_
// Big endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	INT8	ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcHigVal ;
		UINT8	UcLowVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa3 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa0 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlHigVal ;
		UINT32	UlLowVal ;
	} StUllnVal ;
} ;


// Float Data Union
union	FLTVAL {
	float	SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StFltVal ;
} ;

#else	// BIG_ENDDIAN
// Little endian
// Word Data Union
union	WRDVAL{
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	struct {
		UINT8	UcLowVal ;
		UINT8	UcHigVal ;
	} StWrdVal ;
} ;

typedef union WRDVAL	UnWrdVal ;

union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa0 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa3 ;
	} StCdwVal ;
} ;

typedef union DWDVAL	UnDwdVal;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlLowVal ;
		UINT32	UlHigVal ;
	} StUllnVal ;
} ;

typedef union ULLNVAL	UnllnVal;


// Float Data Union
union	FLTVAL {
	float	SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StFltVal ;
} ;
#endif	// _BIG_ENDIAN_

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;


typedef struct STADJPAR {
	struct {
		UINT32	UlAdjPhs ;				//!< Hall Adjust Phase

		UINT16	UsHlxCna ;				//!< Hall Center Value after Hall Adjust
		UINT16	UsHlxMax ;				//!< Hall Max Value
		UINT16	UsHlxMxa ;				//!< Hall Max Value after Hall Adjust
		UINT16	UsHlxMin ;				//!< Hall Min Value
		UINT16	UsHlxMna ;				//!< Hall Min Value after Hall Adjust
		UINT16	UsHlxGan ;				//!< Hall Gain Value
		UINT16	UsHlxOff ;				//!< Hall Offset Value
		UINT16	UsAdxOff ;				//!< Hall A/D Offset Value
		UINT16	UsHlxCen ;				//!< Hall Center Value

		UINT16	UsHlyCna ;				//!< Hall Center Value after Hall Adjust
		UINT16	UsHlyMax ;				//!< Hall Max Value
		UINT16	UsHlyMxa ;				//!< Hall Max Value after Hall Adjust
		UINT16	UsHlyMin ;				//!< Hall Min Value
		UINT16	UsHlyMna ;				//!< Hall Min Value after Hall Adjust
		UINT16	UsHlyGan ;				//!< Hall Gain Value
		UINT16	UsHlyOff ;				//!< Hall Offset Value
		UINT16	UsAdyOff ;				//!< Hall A/D Offset Value
		UINT16	UsHlyCen ;				//!< Hall Center Value

#ifdef	SEL_CLOSED_AF
		UINT16	UsHlzCna ;				//!< Z Hall Center Value after Hall Adjust
		UINT16	UsHlzMax ;				//!< Z Hall Max Value
		UINT16	UsHlzMxa ;				//!< Z Hall Max Value after Hall Adjust
		UINT16	UsHlzMin ;				//!< Z Hall Min Value
		UINT16	UsHlzMna ;				//!< Z Hall Min Value after Hall Adjust
		UINT16	UsHlzGan ;				//!< Z Hall Gain Value
		UINT16	UsHlzOff ;				//!< Z Hall Offset Value
		UINT16	UsAdzOff ;				//!< Z Hall A/D Offset Value
		UINT16	UsHlzCen ;				//!< Z Hall Center Value
		UINT16	UsHlzAmp ;				//!< Z Hall Amp Magnification
#endif
	} StHalAdj ;

	struct {
		UINT32	UlLxgVal ;				//!< Loop Gain X
		UINT32	UlLygVal ;				//!< Loop Gain Y
#ifdef	SEL_CLOSED_AF
		UINT32	UlLzgVal ;				//!< Loop Gain Z
#endif
	} StLopGan ;

	struct {
		UINT16	UsGxoVal ;				//!< Gyro A/D Offset X
		UINT16	UsGyoVal ;				//!< Gyro A/D Offset Y
		UINT16	UsGxoSts ;				//!< Gyro Offset X Status
		UINT16	UsGyoSts ;				//!< Gyro Offset Y Status
	} StGvcOff ;
} stAdjPar ;

__OIS_CMD_HEADER__	stAdjPar	StAdjPar ;		//!< Calibration data

typedef struct STHALLINEAR {
	UINT16	XCoefA[6] ;
	UINT16	XCoefB[6] ;
	UINT16	XZone[5] ;
	UINT16	YCoefA[6] ;
	UINT16	YCoefB[6] ;
	UINT16	YZone[5] ;
} stHalLinear ;

//	for RtnCen
#define		BOTH_ON			0x00
#define		XONLY_ON		0x01
#define		YONLY_ON		0x02
#define		BOTH_OFF		0x03
#define		ZONLY_OFF		0x04
#define		ZONLY_ON		0x05
//	for SetSinWavePara
#define		SINEWAVE		0
#define		XHALWAVE		1
#define		YHALWAVE		2
#define		ZHALWAVE		3
#define		XACTTEST		10
#define		YACTTEST		11
#define		CIRCWAVE		255
//	for TnePtp
#define		HALL_H_VAL		0x3F800000		//!< 1.0
//	for TneCen
#define		OFFDAC_8BIT		0				//!< 8bit Offset DAC select
#define		OFFDAC_3BIT		1				//!< 3bit Offset DAC select
#define		PTP_BEFORE		0
#define		PTP_AFTER		1
#define		PTP_ACCEPT		2
//	for RunHea
#define		ACT_CHK_LVL		0x33320000		//!< 0.4
#define		ACT_CHK_FRQ		0x00068C16		//!< 4Hz
#define		ACT_CHK_NUM		5005			//!< 20.0195/0.004 < x
#define		ACT_THR			0x0A3D0000		//!< 20dB 0.4*10^(-20dB/20)*FFFF
//	for RunGea
// #define		GEA_DIF_HIG		0x0062			//!< 2021_32.8lsb/‹/s    max 3.0‹/s-p-p
#define		GEA_DIF_HIG		0x0057			//!< 2030_87.5lsb/‹/s    max 1.0‹/s-p-p
#define		GEA_DIF_LOW		0x0001
// for RunGea2
// level of judgement
#define		GEA_MAX_LVL		0x0A41			//!< 2030_87.5lsb/‹/s    max 30‹/s-p-p
#define		GEA_MIN_LVL		0x1482			//!< 2030_87.5lsb/‹/s    min 60‹/s-p-p
// mode
#define		GEA_MINMAX_MODE	0x00			//!< min, max mode
#define		GEA_MEAN_MODE	0x01			//!< mean mode

// for FRA measurement
#ifdef		USE_FRA
 #include	"OisFRA.h"
#endif

#endif /* #ifndef OIS_H_ */
