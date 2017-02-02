/**
 * @brief		FRA measurement command for LC898123 F40
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 *
 * @file		OisFRA.c
 * @date		svn:$Date:: 2016-06-17 16:42:32 +0900#$
 * @version	svn:$Revision: 54 $
 * @attention
 **/

//**************************
//	Include Header File
//**************************
#define		__OISFRA__

#include	<math.h>
#include	"Ois.h"

/**
 *
 * The defines meaning follow block.
 *
 *                 RAM_*SIN
 *                    |
 *                    V
 *IN---------Å†-------Åõ--------Å†----------OUT
 *            |                  |
 *            V                  V
 *         RAM_*IN1           RAM_*IN2
 **/

#define		RAM_XSIN	HALL_FRA_XSININ
#define		RAM_XIN1	HALL_FRA_XHOUTB
#define		RAM_XIN2	HALL_FRA_XHOUTA

#define		RAM_YSIN	HALL_FRA_YSININ
#define		RAM_YIN1	HALL_FRA_YHOUTB
#define		RAM_YIN2	HALL_FRA_YHOUTA

#define		RAM_ZSIN	CLAF_RAMA_AFSINE
#define		RAM_ZIN1	CLAF_RAMA_AFDEV
#define		RAM_ZIN2	CLAF_DELAY_AFDZ0

//#define		LPF_ENA		// Enable LPF PASS filter
							// Disable BPF PASS filter


//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */
extern	void RamWrite32A( INT32, INT32 );
extern 	void RamRead32A( UINT16, void * );
/* for Wait timer [Need to adjust for your system] */
extern void	WitTim( UINT16 );

//**************************
//	Local Function Prototype
//**************************
void	FRA_Avg( void ) ;
void	FRA_Calc( StFRAMes_t *, UINT8 ) ;
UINT32	Freq_Convert( float ) ;

//**************************
//	External Function Prototype
//**************************
extern void	SetSineWave(   UINT8 , UINT8 );
extern void	SetSinWavGenInt( void );
extern void	SetTransDataAdr( UINT16, UINT32  ) ;
extern void	MeasureWait( void ) ;
extern void	ClrMesFil( void ) ;
extern void	SetWaitTime( UINT16 ) ;

//********************************************************************************
// Function Name 	: FRA_Avg
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Average of Gain & Phase Calculation
// History			: First edition
//********************************************************************************
void	FRA_Avg( void )
{
	UINT8	i, j ;
	float	SfTempGain, SfTempPhase ;

	// bubble sort
	for( i = 0; i < StFRAParam.StHostCom.UcAvgCycl - 1; i++ ) {
		for( j = i + 1; j < StFRAParam.StHostCom.UcAvgCycl; j++ ) {
			if( StFRAParam.SfGain[ j ]	< StFRAParam.SfGain[ i ] ) {
				SfTempGain				= StFRAParam.SfGain[ i ] ;
				StFRAParam.SfGain[ i ]	= StFRAParam.SfGain[ j ] ;
				StFRAParam.SfGain[ j ]	= SfTempGain ;
			}

			if( StFRAParam.SfPhase[ j ] < StFRAParam.SfPhase[ i ] ) {
				SfTempPhase				= StFRAParam.SfPhase[ i ] ;
				StFRAParam.SfPhase[ i ]	= StFRAParam.SfPhase[ j ] ;
				StFRAParam.SfPhase[ j ]	= SfTempPhase ;
			}
		}
	}

	// Gain is central value
	if( StFRAParam.StHostCom.UcAvgCycl & 0x01 ) {
		i = StFRAParam.StHostCom.UcAvgCycl / 2 ;
	} else {
		i= ( StFRAParam.StHostCom.UcAvgCycl / 2 ) - 1 ;
	}

	StFRAParam.StMesRslt.SfGainAvg	= StFRAParam.SfGain[ i ] ;
	StFRAParam.StMesRslt.SfPhaseAvg	= StFRAParam.SfPhase[ i ] ;

	return ;
}



//********************************************************************************
// Function Name 	: FRA_Calc
// Retun Value		: NON
// Argment Value	: Measurement Results, Measurement Count
// Explanation		: Gain & Phase Calculation
// History			: First edition
//********************************************************************************
void	FRA_Calc( StFRAMes_t *PtFRAMes, UINT8 UcMesCnt )
{
	float	SfTempGain, SfTempPhase ;

	SfTempGain	= ( float )PtFRAMes->UllCumulAdd1 / ( float )PtFRAMes->UllCumulAdd2 ;
	StFRAParam.SfGain[ UcMesCnt ]	= 20.0F * log10f( SfTempGain ) ;

	SfTempPhase	= StFRAParam.StHostCom.SfFrqCom.SfFltVal / FS_FREQ ;
	StFRAParam.SfPhase[ UcMesCnt ]	= SfTempPhase * (( float )PtFRAMes->UsFsCount * 360.0F) ;

	return ;
}



//********************************************************************************
// Function Name 	: Freq_Convert
// Retun Value		: Phase Step Value
// Argment Value	: Frequency
// Explanation		: Convert Frequency
// History			: First edition
//********************************************************************************
UINT32	Freq_Convert( float SfFreq )
{
	UINT32	UlPhsStep ;

	UlPhsStep	= ( UINT32 )( ( SfFreq * ( float )0x100000000 / FS_FREQ + 0.5F ) / 2.0F ) ;

	return( UlPhsStep ) ;
}



//********************************************************************************
// Function Name 	: MesStart_FRA_Single
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	MesStart_FRA_Single( UINT8	UcSingleSweepSel, UINT8	UcDirSel )
{
	UnllnVal		StMeasValueA, StMeasValueB ;
	INT32			SlMeasureParameterA, SlMeasureParameterB ;
	float			SfTmp ;
	UINT8	i ;
	UINT32	UlSampleNum, UlReadVal ;

	if( UcDirSel == X_DIR ) {																		// X axis
		SlMeasureParameterA		=	RAM_XIN1 ;														// Set Measure RAM Address
		SlMeasureParameterB		=	RAM_XIN2 ;														// Set Measure RAM Address
	} else if( UcDirSel == Y_DIR ) {																// Y axis
		SlMeasureParameterA		=	RAM_YIN1 ;														// Set Measure RAM Address
		SlMeasureParameterB		=	RAM_YIN2 ;														// Set Measure RAM Address
#ifdef	SEL_CLOSED_AF
	} else {																						// Z axis
		SlMeasureParameterA		=	RAM_ZIN1 ;														// Set Measure RAM Address
		SlMeasureParameterB		=	RAM_ZIN2 ;														// Set Measure RAM Address
#endif
	}

	SetSinWavGenInt() ;

	RamWrite32A( SinWave_Offset,	Freq_Convert( StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ;		// Freq Setting = Freq * 80000000h / Fs	: 10Hz

	SfTmp	= StFRAParam.StHostCom.SfAmpCom.SfFltVal / 1400.0F ;
	RamWrite32A( SinWave_Gain,		( UINT32 )( ( float )0x7FFFFFFF * SfTmp ) ) ;					// Set Sine Wave Gain

	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)RAM_XSIN ) ;								// Set Sine Wave Input RAM
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)RAM_YSIN ) ;								// Set Sine Wave Input RAM
#ifdef	SEL_CLOSED_AF
	}else{
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)RAM_ZSIN ) ;								// Set Sine Wave Input RAM
#endif
	}

	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;												// Sine Wave Start

	// Change filter to Band pass from LPF
#if LPF_ENA
	// Calculate coefficient a for HPF a coefficient
	UlReadVal	= ( UINT32 )( ( float )0x7FFFFFFF * ( FS_FREQ / ( FS_FREQ + M_PI * StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ) ;

	// Calculate coefficient c for LPF 6dB up
	UlSampleNum	= 2 * UlReadVal - 0x7FFFFFFF ;
	RamWrite32A ( MeasureFilterA_Coeff_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2, UlSampleNum ) ;

	// Calculate 1st IIR coefficient a & b for LPF
	UlSampleNum	= 0x7FFFFFFF - UlReadVal ;
	RamWrite32A ( MeasureFilterA_Coeff_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1, UlSampleNum ) ;

	// Calculate 2ndt IIR coefficient a & b for LPF 6dB up
	UlSampleNum	= 2 * ( 0x7FFFFFFF - UlReadVal ) ;
	RamWrite32A ( MeasureFilterA_Coeff_a2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_a2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2, UlSampleNum ) ;

	ClrMesFil() ;																					// Clear Delay Ram
	SetWaitTime( 500 ) ;

#else	// LPF_ENA
	// Calculate coefficient a for HPF
	UlReadVal	= ( UINT32 )( ( float )0x7FFFFFFF * ( StFRAParam.StHostCom.SfFrqCom.SfFltVal / (StFRAParam.StHostCom.SfFrqCom.SfFltVal + FS_FREQ /  M_PI ) ) ) ;

	// Calculate coefficient c for HPF and LPF
	UlSampleNum	= 0x7FFFFFFF - ( 2 * UlReadVal ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2, UlSampleNum ) ;

	// Calculate coefficient a & b for LPF
	UlSampleNum	= UlReadVal ;
	RamWrite32A ( MeasureFilterA_Coeff_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1, UlSampleNum ) ;

	// Calculate coefficient a & b for HPF
	UlSampleNum	= ( 0x7FFFFFFF - UlReadVal ) ;
	RamWrite32A ( MeasureFilterA_Coeff_a2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_a2, UlSampleNum ) ;
	UlSampleNum = ~UlSampleNum + 1;
	RamWrite32A ( MeasureFilterA_Coeff_b2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2, UlSampleNum ) ;

	ClrMesFil() ;																					// Clear Delay Ram
	SetWaitTime( 8000 / StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ;
#endif	// LPF_ENA


	if( UcSingleSweepSel ) {
		RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
		UlReadVal	|= 0x00000300 ;																	// SweepMode = 0x03
		RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;									// Set Phase Measure Mode
	}

	for( i = 0 ; i < StFRAParam.StHostCom.UcAvgCycl ; i++ ) {
		RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
		UlReadVal	|= 0x00000001 ;																	// PhaseMode = 0x01
		RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;									// Set Phase Measure Mode
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamWrite32A( StMeasFunc_PMC_SiFsCountF, 0x00000000 ) ;										// Phase Measure Mode Clear
		RamWrite32A( StMeasFunc_PMC_SiFsCountR, 0x00000000 ) ;										// Phase Measure Mode Clear

		SetTransDataAdr( StMeasFunc_MFA_PiMeasureRam1, ( UINT32 )SlMeasureParameterA ) ;			// Set Measure Filter A Ram Address
		SetTransDataAdr( StMeasFunc_MFB_PiMeasureRam2, ( UINT32 )SlMeasureParameterB ) ;			// Set Measure Filter B Ram Address
		RamWrite32A( StMeasFunc_MFA_SiSampleNumA, 0 ) ;												// Clear Measure Counter
		RamWrite32A( StMeasFunc_MFB_SiSampleNumB, 0 ) ;												// Clear Measure Counter

		UlSampleNum	= ( UINT32 )( FS_FREQ / StFRAParam.StHostCom.SfFrqCom.SfFltVal ) + 1 ;

		if( UcSingleSweepSel ) {
			RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
			UlReadVal	&= 0xFFFFFDFF ;																// SweepMode = wait mode on
			RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;
		}

		// === START MEASUREMENT ===
		RamWrite32A( StMeasFunc_MFA_SiSampleMaxA, UlSampleNum ) ;									// Set Measure Max Number
		RamWrite32A( StMeasFunc_MFB_SiSampleMaxB, UlSampleNum ) ;									// Set Measure Max Number

#if !LPF_ENA
		if( i == 0)
			WitTim( 8000 / StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ;
#endif
		MeasureWait() ;

		RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;			// X axis
		RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;			// Y axis
		RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;

		StFRAMes.UllCumulAdd1	= StMeasValueA.UllnValue ;
		StFRAMes.UllCumulAdd2	= StMeasValueB.UllnValue ;
		RamRead32A( StMeasFunc_PMC_SiFsCountF, &UlReadVal ) ;
		StFRAMes.UsFsCount		= UlReadVal ;

		FRA_Calc( &StFRAMes, i ) ;
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2 + 4, 0x00000000 ) ;								// Integral Value Clear
		if( !UcSingleSweepSel ) {
			RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, 0x00000000 ) ;								// Phase Measure Mode Clear
		} else {
			RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
			UlReadVal	&= 0x00000300 ;																// clear other SweepMode
			RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;								// Set Phase Measure Mode
		}
		RamWrite32A( StMeasFunc_PMC_UcCrossDetectA, 0x00000000 ) ;									// Phase Measure Mode Clear
	}

	if( !UcSingleSweepSel ) {																		// Stop sine wave when single mode
		RamWrite32A( SinWaveC_Regsiter,		0x00000000 ) ;											// Sine Wave Stop
		SetTransDataAdr( SinWave_OutAddr,	( UINT32 )0x00000000 ) ;								// Set Sine Wave Input RAM


		if( UcDirSel == X_DIR ) {
			RamWrite32A( RAM_XSIN,	0x00000000 ) ;													// DelayRam Clear
		} else if( UcDirSel == Y_DIR ) {
			RamWrite32A( RAM_YSIN,	0x00000000 ) ;													// DelayRam Clear
#ifdef	SEL_CLOSED_AF
		} else {
			RamWrite32A( RAM_ZSIN,	0x00000000 ) ;													// DelayRam Clear
#endif
		}
	}

	FRA_Avg() ;

	if( StFRAParam.StMesRslt.SfPhaseAvg > 180.0F ) {
		StFRAParam.StMesRslt.SfPhaseAvg	-= 360.0F ;
	} else if( StFRAParam.StMesRslt.SfPhaseAvg < -180.0F ) {
		StFRAParam.StMesRslt.SfPhaseAvg	+= 360.0F ;
	}
	StFRAParam.StMesRslt.SfPhaseAvg	= fmod(StFRAParam.StMesRslt.SfPhaseAvg, 180.0) ;
}



//********************************************************************************
// Function Name 	: MesStart_FRA_Continue
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Continue Measurement Function
// History			: First edition
//********************************************************************************
void	MesStart_FRA_Continue( void )
{
	UnllnVal	StMeasValueA, StMeasValueB ;
	UINT8		i ;
	UINT32		UlSampleNum, UlReadVal ;

	RamWrite32A( SinWave_Offset,	Freq_Convert( StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ;

	// Change filter to Band pass from LPF
#if LPF_ENA
	// Calculate coefficient a for HPF
	UlReadVal	= ( UINT32 )( ( float )0x7FFFFFFF * ( FS_FREQ / ( FS_FREQ + M_PI * StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ) ;

	// Calculate coefficient c for HPF
	UlSampleNum	= 2 * UlReadVal - 0x7FFFFFFF ;
	RamWrite32A ( MeasureFilterA_Temp_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Temp_c2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_c2, UlSampleNum ) ;

	// Calculate coefficient a & b for LPF
	UlSampleNum	= 0x7FFFFFFF - UlReadVal ;
	RamWrite32A ( MeasureFilterA_Temp_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Temp_b1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_b1, UlSampleNum ) ;

	UlSampleNum	= 2 * ( 0x7FFFFFFF - UlReadVal ) ;
	RamWrite32A ( MeasureFilterA_Temp_a2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_a2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Temp_b2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_b2, UlSampleNum ) ;

	ClrMesFil() ;																					// Clear Delay Ram
	SetWaitTime( 500 ) ;

#else	// LPF_ENA
	// Calculate coefficient a for HPF
	UlReadVal	= ( UINT32 )( ( float )0x7FFFFFFF * ( StFRAParam.StHostCom.SfFrqCom.SfFltVal / (StFRAParam.StHostCom.SfFrqCom.SfFltVal + FS_FREQ /  M_PI ) ) ) ;

	// Calculate coefficient c for HPF and LPF
	UlSampleNum	= 0x7FFFFFFF - ( 2 * UlReadVal ) ;
	RamWrite32A ( MeasureFilterA_Temp_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_c1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Temp_c2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_c2, UlSampleNum ) ;

	// Calculate coefficient a & b for LPF
	UlSampleNum	= UlReadVal ;
	RamWrite32A ( MeasureFilterA_Temp_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_a1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterA_Temp_b1, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_b1, UlSampleNum ) ;

	// Calculate coefficient a & b for HPF
	UlSampleNum	= ( 0x7FFFFFFF - UlReadVal ) ;
	RamWrite32A ( MeasureFilterA_Temp_a2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_a2, UlSampleNum ) ;

	UlSampleNum = ~UlSampleNum + 1;
	RamWrite32A ( MeasureFilterA_Temp_b2, UlSampleNum ) ;
	RamWrite32A ( MeasureFilterB_Temp_b2, UlSampleNum ) ;

	ClrMesFil() ;																					// Clear Delay Ram
	SetWaitTime( 8000 / StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ;
#endif	// LPF_ENA

	RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
	UlReadVal	|= 0x00000400 ;
	RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;

	for( i = 0 ; i < StFRAParam.StHostCom.UcAvgCycl ; i++ ) {
		RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
		UlReadVal	|= 0x00000001 ;
		RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;									// Set Phase Measure Mode
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamWrite32A( StMeasFunc_PMC_SiFsCountF, 0x00000000 ) ;										// Phase Measure Mode Clear
		RamWrite32A( StMeasFunc_PMC_SiFsCountR, 0x00000000 ) ;										// Phase Measure Mode Clear

		RamWrite32A( StMeasFunc_MFA_SiSampleNumA, 0 ) ;												// Clear Measure Counter
		RamWrite32A( StMeasFunc_MFB_SiSampleNumB, 0 ) ;												// Clear Measure Counter

		UlSampleNum	= ( UINT32 )( FS_FREQ / StFRAParam.StHostCom.SfFrqCom.SfFltVal ) + 1 ;

		RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
		UlReadVal	&= 0xFFFFFDFF ;
		RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;

		RamWrite32A( StMeasFunc_MFA_SiSampleMaxA, UlSampleNum ) ;									// Set Measure Max Number
		RamWrite32A( StMeasFunc_MFB_SiSampleMaxB, UlSampleNum ) ;									// Set Measure Max Number
#if !LPF_ENA
		if( i == 0)
			WitTim( 8000 / StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ;
#endif	// !LPF_ENA

		MeasureWait() ;

		RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;			// X axis
		RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;			// Y axis
		RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;

		StFRAMes.UllCumulAdd1	= StMeasValueA.UllnValue ;
		StFRAMes.UllCumulAdd2	= StMeasValueB.UllnValue ;
		RamRead32A( StMeasFunc_PMC_SiFsCountF, &UlReadVal ) ;
		StFRAMes.UsFsCount		= UlReadVal ;

		FRA_Calc( &StFRAMes, i ) ;
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFA_LLiAbsInteg1 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2, 0x00000000 ) ;									// Integral Value Clear
		RamWrite32A( StMeasFunc_MFB_LLiAbsInteg2 + 4, 0x00000000 ) ;								// Integral Value Clear
		RamRead32A( StMeasFunc_PMC_UcPhaseMesMode, &UlReadVal ) ;
		UlReadVal	&= 0x00000300 ;
		RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, UlReadVal ) ;									// Set Phase Measure Mode
		RamWrite32A( StMeasFunc_PMC_UcCrossDetectA, 0x00000000 ) ;									// Phase Measure Mode Clear
	}

	FRA_Avg() ;

	if( StFRAParam.StMesRslt.SfPhaseAvg > 180.0F ) {
		StFRAParam.StMesRslt.SfPhaseAvg	-= 360.0F ;
	} else if( StFRAParam.StMesRslt.SfPhaseAvg < -180.0F ) {
		StFRAParam.StMesRslt.SfPhaseAvg	+= 360.0F ;
	}
	StFRAParam.StMesRslt.SfPhaseAvg	= fmod( StFRAParam.StMesRslt.SfPhaseAvg, 180.0);
}



//********************************************************************************
// Function Name 	: MesEnd_FRA_Sweep
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stop Measurement Function
// History			: First edition
//********************************************************************************
void	MesEnd_FRA_Sweep( void )
{
	// Stop Measure Filter
	RamWrite32A( StMeasFunc_PMC_UcPhaseMesMode, 0 ) ;
	RamWrite32A( StMeasFunc_PMC_UcCrossDetectA, 0x00000000 ) ;										// Phase Measure Mode Clear
	RamWrite32A( StMeasFunc_MFA_SiSampleMaxA, 0 ) ;
	RamWrite32A( StMeasFunc_MFB_SiSampleMaxB, 0 ) ;
	// Stop Sine Wave
	RamWrite32A( SinWaveC_Regsiter,		0x00000000 ) ;												// Sine Wave Stop
	SetTransDataAdr( SinWave_OutAddr,	( UINT32 )0x00000000 ) ;									// Set Sine Wave Input RAM


	RamWrite32A( RAM_XSIN,	0x00000000 ) ;															// DelayRam Clear
	RamWrite32A( RAM_YSIN,	0x00000000 ) ;															// DelayRam Clear
#ifdef	SEL_CLOSED_AF
	RamWrite32A( RAM_ZSIN,	0x00000000 ) ;															// DelayRam Clear
#endif

	return ;

}
