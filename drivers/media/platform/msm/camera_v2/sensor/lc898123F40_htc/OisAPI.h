/**
 * @brief		OIS system header for LC898123
 * 				API List for customers
 *
 * @author		Copyright (C) 2015, ON Semiconductor, all right reserved.
 *
 * @file		OisAPI.h
 * @date		svn:$Date:: 2016-06-17 16:42:32 +0900#$
 * @version		svn:$Revision: 54 $
 * @attention
 **/
#ifndef OISAPI_H_
#define OISAPI_H_
#include	"MeasurementLibrary.h"

//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISCMD__
	#define	__OIS_CMD_HEADER__
#else
	#define	__OIS_CMD_HEADER__		extern
#endif

#ifdef	__OISFLSH__
	#define	__OIS_FLSH_HEADER__
#else
	#define	__OIS_FLSH_HEADER__		extern
#endif

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
#define			__OIS_MODULE_CALIBRATION__		//!< for module maker to done the calibration.
//#define 		__CRC_VERIFY__					//!< select CRC16 for upload verify, if this comment out, MD5 is selected.
//#define		__OIS_BIG_ENDIAN__				//!< endian of MPU

//#define		__OIS_CLOSED_AF__

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct STRECALIB {
	INT16	SsFctryOffX ;
	INT16	SsFctryOffY ;
	INT16	SsRecalOffX ;
	INT16	SsRecalOffY ;
	INT16	SsDiffX ;
	INT16	SsDiffY ;
} stReCalib ;

//****************************************************
//	API LIST
//****************************************************
/* Status Read and OIS enable [mandatory] */
__OIS_CMD_HEADER__	UINT8	RdStatus( UINT8 ) ;						//!< Status Read whether initialization finish or not.
__OIS_CMD_HEADER__	void	OisEna( void ) ;						//!< OIS Enable function
__OIS_CMD_HEADER__	void	OisDis( void ) ;						//!< OIS Disable function

/* Others [option] */
__OIS_CMD_HEADER__	UINT8	RtnCen( UINT8 ) ;						//!< Return to center function. Hall servo on/off
__OIS_CMD_HEADER__	void	OisEnaNCL( void ) ;						//!< OIS Enable function w/o delay clear
__OIS_CMD_HEADER__	void	OisEnaDrCl( void ) ;					//!< OIS Enable function force drift cancel
__OIS_CMD_HEADER__	void	OisEnaDrNcl( void ) ;					//!< OIS Enable function w/o delay clear and force drift cancel
__OIS_CMD_HEADER__	void	SetRec( void ) ;						//!< Change to recording mode function
__OIS_CMD_HEADER__	void	SetStill( void ) ;						//!< Change to still mode function

__OIS_CMD_HEADER__	void	SetPanTiltMode( UINT8 ) ;				//!< Pan/Tilt control (default ON)
//__OIS_CMD_HEADER__	void	RdHallCalData( void ) ;					//!< Read Hall Calibration Data in Data Ram

__OIS_CMD_HEADER__	UINT8	RunHea( void ) ;						//!< Hall Examination of Acceptance
__OIS_CMD_HEADER__	UINT8	RunGea( void ) ;						//!< Gyro Examination of Acceptance
__OIS_CMD_HEADER__	UINT8	RunGea2( UINT8 ) ;						//!< Gyro Examination of Acceptance


__OIS_CMD_HEADER__	void	OscStb( void );							//!< Standby the oscillator
__OIS_CMD_HEADER__	UINT8	GyroReCalib( stReCalib * ) ;			//!< Gyro offset re-calibration
__OIS_CMD_HEADER__	UINT32	ReadCalibID( void ) ;					//!< Read calibration ID
//__OIS_CMD_HEADER__	UINT16	GyrSlf( void ) ;						//!< Gyro self test

#ifdef	__OIS_MODULE_CALIBRATION__

 /* Calibration Main [mandatory] */
 #ifdef	__OIS_CLOSED_AF__
 __OIS_CMD_HEADER__	UINT32	TneRunA( void ) ;						//!< calibration with close AF
 __OIS_CMD_HEADER__	UINT32	AFHallAmp( void ) ;

 #else
 __OIS_CMD_HEADER__	UINT32	TneRun( void );							//!< calibration for bi-direction AF
 #endif
// __OIS_CMD_HEADER__	UINT8	TneADO( ) ;

 __OIS_CMD_HEADER__	void	TneSltPos( UINT8 ) ;					//!< for NVC
 __OIS_CMD_HEADER__	void	TneVrtPos( UINT8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ void	TneHrzPos( UINT8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT16	TneADO( void ) ;
 __OIS_CMD_HEADER__	UINT8	FrqDet( void ) ;

 __OIS_CMD_HEADER__	UINT8	WrHallCalData( void ) ;					//!< upload the calibration data except gyro gain to Flash
 __OIS_CMD_HEADER__	UINT8	WrGyroGainData( void ) ;				//!< upload the gyro gain to Flash
 __OIS_CMD_HEADER__	UINT8	WrGyroAngleData( void ) ;				//!< upload the gyto angle correction to Flash
 __OIS_CMD_HEADER__	UINT8	WrCLAFData( void ) ;					//!< Flash Write CL-AF Calibration Data Function
 __OIS_CMD_HEADER__	UINT8	WrMixingData( void ) ;					//!< Flash Write Mixing Data Function
 __OIS_CMD_HEADER__	UINT8	WrFstData( void ) ;						//!< Flash Write FST calibration data Function
 __OIS_CMD_HEADER__	UINT8	WrMixCalData( UINT8, mlMixingValue * ) ;
 __OIS_CMD_HEADER__	UINT8	WrGyroOffsetData( void ) ;

 #ifdef	HF_LINEAR_ENA
// __OIS_CMD_HEADER__	void	SetHalLnData( UINT16 * );
// __OIS_CMD_HEADER__	INT16	WrHalLnData( UINT8 );
 #endif	// HF_LINEAR_ENA

 #ifdef	HF_MIXING_ENA
 __OIS_CMD_HEADER__	INT8	WrMixCalData( UINT8, mlMixingValue * ) ;//!< upload the mixing coefficient to Flash
 #endif	// HF_MIXING_ENA

 __OIS_CMD_HEADER__	UINT8	WrLinCalData( UINT8, mlLinearityValue * ) ;
 __OIS_CMD_HEADER__	UINT8	ErCalData( UINT16 ) ;

 /* Flash Update */
 __OIS_FLSH_HEADER__	UINT8	ReadWPB( void ) ;						//!< WPB level read
 __OIS_FLSH_HEADER__	UINT8	UnlockCodeSet( void ) ;					//!< <Flash Memory> Unlock Code Set
 __OIS_FLSH_HEADER__	UINT8	UnlockCodeClear(void) ;					//!< <Flash Memory> Clear Unlock Code
 __OIS_FLSH_HEADER__	void	FlashByteRead( UINT32, UINT8 *, UINT8 ) ;
 __OIS_FLSH_HEADER__	void	FlashSectorRead( UINT32, UINT8 * ) ;
 __OIS_FLSH_HEADER__	UINT8	FlashInt32Write( UINT32, UINT32 *, UINT8 ) ;

 __OIS_FLSH_HEADER__	UINT8	FlashBlockErase( UINT32 ) ;
 __OIS_FLSH_HEADER__	UINT8	FlashSectorErase( UINT32 ) ;
 __OIS_FLSH_HEADER__	UINT8	FlashSectorWrite( UINT32, UINT8 * ) ;
 __OIS_FLSH_HEADER__	UINT8	FlashProtectStatus( void ) ;

 __OIS_FLSH_HEADER__	UINT8	FlashUpdateF40( void ) ;				//!< Flash Update for LC898123F40
 __OIS_FLSH_HEADER__	UINT8	FlashUpdateF40ex( UINT8 );				//!< Flash Update for LC898123F40
 __OIS_FLSH_HEADER__	UINT8	EraseCalDataF40( void ) ;				//!< Flash erase calibration data(NVR2)
 __OIS_FLSH_HEADER__	void	ReadCalDataF40( UINT32 *, UINT32 * ) ;	//!< Flash read calibration data(NVR2)
 __OIS_FLSH_HEADER__	UINT8	WriteCalDataF40( UINT32 *, UINT32 * ) ;	//!< Flash write calibration data(NVR2)
 __OIS_FLSH_HEADER__	void	CalcChecksum( const UINT8 *, UINT32, UINT32 *, UINT32 * ) ;

 // following functions are into boot rom mode. if use, be careful.
 __OIS_FLSH_HEADER__	UINT8	FlashSectorRead_Burst( UINT32, UINT8 *, UINT8 ) ;
 __OIS_FLSH_HEADER__	UINT8	FlashSectorWrite_Burst( UINT32, UINT8 *, UINT8 ) ;
 //HTC_START
 __OIS_FLSH_HEADER__	void	FlashSectorRead_htc( UINT32, UINT8 * ) ;
 //HTC_END

#endif	// __OIS_MODULE_CALIBRATION__

#endif /* #ifndef OISAPI_H_ */
