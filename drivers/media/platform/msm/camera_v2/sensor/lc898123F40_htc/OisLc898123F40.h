/**
 * @brief		LC898123F40 Global declaration & prototype declaration
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 *
 * @file		OisLC898123F40.h
 * @date		svn:$Date:: 2016-06-17 16:42:32 +0900#$
 * @version	svn:$Revision: 54 $
 * @attention
 **/

//==============================================================================
// Calibration Data Memory Map
//==============================================================================
// Calibration Status
#define	CALIBRATION_STATUS		(  0 )
// Hall amplitude Calibration X
#define	HALL_MAX_BEFORE_X		(  1 )
#define	HALL_MIN_BEFORE_X		(  2 )
#define	HALL_MAX_AFTER_X		(  3 )
#define	HALL_MIN_AFTER_X		(  4 )
// Hall amplitude Calibration Y
#define	HALL_MAX_BEFORE_Y		(  5 )
#define	HALL_MIN_BEFORE_Y		(  6 )
#define	HALL_MAX_AFTER_Y		(  7 )
#define	HALL_MIN_AFTER_Y		(  8 )
// Hall Bias/Offset
#define	HALL_BIAS_DAC_X			(  9 )
#define	HALL_OFFSET_DAC_X		( 10 )
#define	HALL_BIAS_DAC_Y			( 11 )
#define	HALL_OFFSET_DAC_Y		( 12 )
// Loop Gain Calibration X
#define	LOOP_GAIN_X				( 13 )
// Loop Gain Calibration Y
#define	LOOP_GAIN_Y				( 14 )
// Lens Center Calibration
#define	MECHA_CENTER_X			( 15 )
#define	MECHA_CENTER_Y			( 16 )
// Optical Center Calibration
#define	OPT_CENTER_X			( 17 )
#define	OPT_CENTER_Y			( 18 )
// Gyro Offset Calibration
#define	GYRO_OFFSET_X			( 19 )
#define	GYRO_OFFSET_Y			( 20 )
// Gyro Gain Calibration
#define	GYRO_GAIN_X				( 21 )
#define	GYRO_GAIN_Y				( 22 )
// AF calibration
#ifdef	SEL_CLOSED_AF
#define	AF_HALL_BIAS_DAC		( 23 )
#define	AF_HALL_OFFSET_DAC		( 24 )
#define	AF_LOOP_GAIN			( 25 )
#define	AF_MECHA_CENTER			( 26 )
#define	AF_HALL_AMP_MAG			( 27 )
#define	AF_HALL_MAX_BEFORE		( 28 )
#define	AF_HALL_MIN_BEFORE		( 29 )
#define	AF_HALL_MAX_AFTER		( 30 )
#define	AF_HALL_MIN_AFTER		( 31 )
#else	// SEL_CLOSED_AF
#define	AF_LONG_M_RRMD1			( 23 )
#define	AF_LONG_I_RRMD1			( 24 )
#define	AF_SHORT_IIM_RRMD1		( 25 )
#define	AF_SHORT_IMI_RRMD1		( 26 )
#define	AF_SHORT_MIM_RRMD1		( 27 )
#define	AF_SHORT_MMI_RRMD1		( 28 )
#endif	// SEL_CLOSED_AF
// Gyro mixing correction
#define MIXING_HX45X			( 32 )
#define MIXING_HX45Y			( 33 )
#define MIXING_HY45Y			( 34 )
#define MIXING_HY45X			( 35 )
#define MIXING_HXSX				( 36 )
#define MIXING_HYSX				( 36 )
// Gyro angle correction
#define MIXING_GX45X			( 37 )
#define MIXING_GX45Y			( 38 )
#define MIXING_GY45Y			( 39 )
#define MIXING_GY45X			( 40 )
// Liniearity correction
#define LN_POS1					( 41 )
#define LN_POS2					( 42 )
#define LN_POS3					( 43 )
#define LN_POS4					( 44 )
#define LN_POS5					( 45 )
#define LN_POS6					( 46 )
#define LN_POS7					( 47 )
#define LN_STEP					( 48 )
// Factory Gyro Gain Calibration
#define	GYRO_FCTRY_OFST_X		( 49 )
#define	GYRO_FCTRY_OFST_Y		( 50 )


//==============================================================================
//DMA
//==============================================================================
#define		HALL_RAM_X_COMMON				0x0110
#define			HALL_RAM_HXOFF					0x0000 + HALL_RAM_X_COMMON
#define			HALL_RAM_HXOFF1					0x0004 + HALL_RAM_X_COMMON
#define			HALL_RAM_HXOUT0					0x0008 + HALL_RAM_X_COMMON
#define			HALL_RAM_HXOUT1					0x000C + HALL_RAM_X_COMMON
#define			HALL_RAM_SINDX0					0x0010 + HALL_RAM_X_COMMON
#define			HALL_RAM_HXLOP					0x0014 + HALL_RAM_X_COMMON
#define			HALL_RAM_SINDX1					0x0018 + HALL_RAM_X_COMMON
#define			HALL_RAM_HALL_X_OUT				0x001C + HALL_RAM_X_COMMON
#define		HALL_RAM_HALL_SwitchX			0x015c

#define		HALL_RAM_Y_COMMON				0x0160
#define			HALL_RAM_HYOFF					0x0000 + HALL_RAM_Y_COMMON
#define			HALL_RAM_HYOFF1					0x0004 + HALL_RAM_Y_COMMON
#define			HALL_RAM_HYOUT0					0x0008 + HALL_RAM_Y_COMMON
#define			HALL_RAM_HYOUT1					0x000C + HALL_RAM_Y_COMMON
#define			HALL_RAM_SINDY0					0x0010 + HALL_RAM_Y_COMMON
#define			HALL_RAM_HYLOP					0x0014 + HALL_RAM_Y_COMMON
#define			HALL_RAM_SINDY1					0x0018 + HALL_RAM_Y_COMMON
#define			HALL_RAM_HALL_Y_OUT				0x001C + HALL_RAM_Y_COMMON
#define		HALL_RAM_HALL_SwitchY			0x01AC


#define		HALL_RAM_COMMON					0x01B0
				//  HallFilterDelay.h HALL_RAM_COMMON_t
#define			HALL_RAM_HXIDAT					0x0000 + HALL_RAM_COMMON
#define			HALL_RAM_HYIDAT					0x0004 + HALL_RAM_COMMON
#define			HALL_RAM_GYROX_OUT				0x0008 + HALL_RAM_COMMON
#define			HALL_RAM_GYROY_OUT				0x000C + HALL_RAM_COMMON

#define		GyroFilterDelayX_delay3_2		0x01D0
#define		GyroFilterDelayX_GXH1Z2				0x0000 + GyroFilterDelayX_delay3_2
#define		GyroFilterDelayY_delay3_2		0x01F8
#define		GyroFilterDelayY_GYH1Z2				0x0000 + GyroFilterDelayY_delay3_2

#define		GYRO_RAM_X						0x0210
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROX_OFFSET			0x0000 + GYRO_RAM_X
#define			GYRO_RAM_GX2X4XF_IN				0x0004 + GYRO_RAM_GYROX_OFFSET
#define			GYRO_RAM_GX2X4XF_OUT			0x0004 + GYRO_RAM_GX2X4XF_IN
#define			GYRO_RAM_GXFAST					0x0004 + GYRO_RAM_GX2X4XF_OUT
#define			GYRO_RAM_GXSLOW					0x0004 + GYRO_RAM_GXFAST
#define			GYRO_RAM_GYROX_G1OUT			0x0004 + GYRO_RAM_GXSLOW
#define			GYRO_RAM_GYROX_G2OUT			0x0004 + GYRO_RAM_GYROX_G1OUT
#define			GYRO_RAM_GYROX_G3OUT			0x0004 + GYRO_RAM_GYROX_G2OUT
#define			GYRO_RAM_GYROX_OUT				0x0004 + GYRO_RAM_GYROX_G3OUT
#define		GYRO_RAM_Y						0x0234
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROY_OFFSET			0x0000 + GYRO_RAM_Y
#define			GYRO_RAM_GY2X4XF_IN				0x0004 + GYRO_RAM_GYROY_OFFSET
#define			GYRO_RAM_GY2X4XF_OUT			0x0004 + GYRO_RAM_GY2X4XF_IN
#define			GYRO_RAM_GYFAST					0x0004 + GYRO_RAM_GY2X4XF_OUT
#define			GYRO_RAM_GYSLOW					0x0004 + GYRO_RAM_GYFAST
#define			GYRO_RAM_GYROY_G1OUT			0x0004 + GYRO_RAM_GYSLOW
#define			GYRO_RAM_GYROY_G2OUT			0x0004 + GYRO_RAM_GYROY_G1OUT
#define			GYRO_RAM_GYROY_G3OUT			0x0004 + GYRO_RAM_GYROY_G2OUT
#define			GYRO_RAM_GYROY_OUT				0x0004 + GYRO_RAM_GYROY_G3OUT
#define		GYRO_RAM_COMMON					0x0258
				// GyroFilterDelay.h GYRO_RAM_COMMON_t
#define			GYRO_RAM_GX_ADIDAT				0x0000 + GYRO_RAM_COMMON
#define			GYRO_RAM_GY_ADIDAT				0x0004 + GYRO_RAM_GX_ADIDAT
#define			GYRO_RAM_SINDX					0x0004 + GYRO_RAM_GY_ADIDAT
#define			GYRO_RAM_SINDY					0x0004 + GYRO_RAM_SINDX
#define			GYRO_RAM_GXLENSZ				0x0004 + GYRO_RAM_SINDY
#define			GYRO_RAM_GYLENSZ				0x0004 + GYRO_RAM_GXLENSZ
#define			GYRO_RAM_GXOX_OUT				0x0004 + GYRO_RAM_GYLENSZ
#define			GYRO_RAM_GYOX_OUT				0x0004 + GYRO_RAM_GXOX_OUT
#define			GYRO_RAM_GXOFFZ					0x0004 + GYRO_RAM_GYOX_OUT
#define			GYRO_RAM_GYOFFZ					0x0004 + GYRO_RAM_GXOFFZ
#define			GYRO_RAM_LIMITX					0x0004 + GYRO_RAM_GYOFFZ
#define			GYRO_RAM_LIMITY					0x0004 + GYRO_RAM_LIMITX
#define			GYRO_RAM_GYROX_AFCnt			0x0004 + GYRO_RAM_LIMITY
#define			GYRO_RAM_GYROY_AFCnt			0x0004 + GYRO_RAM_GYROX_AFCnt
#define			GYRO_RAM_GYRO_Switch			0x0004 + GYRO_RAM_GYROY_AFCnt		// 1Byte
#define			GYRO_RAM_GYRO_AF_Switch			0x0001 + GYRO_RAM_GYRO_Switch		// 1Byte

#define		StMeasureFunc					0x02B0
				// MeasureFilter.h	MeasureFunction_Type
#define			StMeasFunc_SiSampleNum			0x0000 + StMeasureFunc					//
#define			StMeasFunc_SiSampleMax			0x0004 + StMeasFunc_SiSampleNum			//

#define		StMeasureFunc_MFA				0x02B8
#define			StMeasFunc_MFA_SiMax1			0x0000 + StMeasureFunc_MFA
#define			StMeasFunc_MFA_SiMin1			0x0004 + StMeasFunc_MFA_SiMax1
#define			StMeasFunc_MFA_UiAmp1			0x0004 + StMeasFunc_MFA_SiMin1
#define			StMeasFunc_MFA_UiDUMMY1			0x0004 + StMeasFunc_MFA_UiAmp1
#define			StMeasFunc_MFA_LLiIntegral1		0x0004 + StMeasFunc_MFA_UiDUMMY1
#define			StMeasFunc_MFA_LLiAbsInteg1		0x0008 + StMeasFunc_MFA_LLiIntegral1
#define			StMeasFunc_MFA_PiMeasureRam1	0x0008 + StMeasFunc_MFA_LLiAbsInteg1

#define		StMeasureFunc_MFB				0x02E0
#define			StMeasFunc_MFB_SiMax2			0x0000 + StMeasureFunc_MFB
#define			StMeasFunc_MFB_SiMin2			0x0004 + StMeasFunc_MFB_SiMax2
#define			StMeasFunc_MFB_UiAmp2			0x0004 + StMeasFunc_MFB_SiMin2
#define			StMeasFunc_MFB_UiDUMMY1			0x0004 + StMeasFunc_MFB_UiAmp2
#define			StMeasFunc_MFB_LLiIntegral2		0x0004 + StMeasFunc_MFB_UiDUMMY1
#define			StMeasFunc_MFB_LLiAbsInteg2		0x0008 + StMeasFunc_MFB_LLiIntegral2
#define			StMeasFunc_MFB_PiMeasureRam2	0x0008 + StMeasFunc_MFB_LLiAbsInteg2

#define		MeasureFilterA_Delay			0x0308
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterA_Delay_z11		0x0000 + MeasureFilterA_Delay
#define			MeasureFilterA_Delay_z12		0x0004 + MeasureFilterA_Delay_z11
#define			MeasureFilterA_Delay_z21		0x0004 + MeasureFilterA_Delay_z12
#define			MeasureFilterA_Delay_z22		0x0004 + MeasureFilterA_Delay_z21

#define		MeasureFilterB_Delay			0x0318
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterB_Delay_z11		0x0000 + MeasureFilterB_Delay
#define			MeasureFilterB_Delay_z12		0x0004 + MeasureFilterB_Delay_z11
#define			MeasureFilterB_Delay_z21		0x0004 + MeasureFilterB_Delay_z12
#define			MeasureFilterB_Delay_z22		0x0004 + MeasureFilterB_Delay_z21

#define		SinWaveC						0x0328
#define			SinWaveC_Pt						0x0000 + SinWaveC
#define			SinWaveC_Regsiter				0x0004 + SinWaveC_Pt
//#define			SinWaveC_SignFlag				0x0004 + SinWaveC_Regsiter

#define		SinWave							0x0334
				// SinGenerator.h SinWave_t
#define			SinWave_Offset					0x0000 + SinWave
#define			SinWave_Phase					0x0004 + SinWave_Offset
#define			SinWave_Gain					0x0004 + SinWave_Phase
#define			SinWave_Output					0x0004 + SinWave_Gain
#define			SinWave_OutAddr					0x0004 + SinWave_Output
#define		CosWave							0x0348
				// SinGenerator.h SinWave_t
#define			CosWave_Offset					0x0000 + CosWave
#define			CosWave_Phase					0x0004 + CosWave_Offset
#define			CosWave_Gain					0x0004 + CosWave_Phase
#define			CosWave_Output					0x0004 + CosWave_Gain
#define			CosWave_OutAddr					0x0004 + CosWave_Output

#define		WaitTimerData					0x035C
				// CommonLibrary.h  WaitTimer_Type
#define			WaitTimerData_UiWaitCounter		0x0000 + WaitTimerData					// 0x035C
#define			WaitTimerData_UiTargetCount		0x0004 + WaitTimerData_UiWaitCounter	// 0x0360

#define		PanTilt_DMA						0x0370
#define			PanTilt_DMA_ScTpdSts			0x000C + PanTilt_DMA

#define		HALL_FRA_X_COMMON				0x049C
#define			HALL_FRA_XSININ					0x0000 + HALL_FRA_X_COMMON
#define			HALL_FRA_XHOUTB					0x0004 + HALL_FRA_XSININ
#define			HALL_FRA_XHOUTA					0x0004 + HALL_FRA_XHOUTB

#define		HALL_FRA_Y_COMMON				0x04A8
#define			HALL_FRA_YSININ					0x0000 + HALL_FRA_Y_COMMON
#define			HALL_FRA_YHOUTB					0x0004 + HALL_FRA_YSININ
#define			HALL_FRA_YHOUTA					0x0004 + HALL_FRA_YHOUTB

#define		StMeasureFunc_PMC				0x04B4
#define			StMeasFunc_PMC_UcPhaseMesMode	0x0000 + StMeasureFunc_PMC				// 0x04B4
#define			StMeasFunc_PMC_UcFRASweepMode	0x0001 + StMeasFunc_PMC_UcPhaseMesMode	// 0x04B5
#define			StMeasFunc_PMC_UcPrevSign_A		0x0001 + StMeasFunc_PMC_UcFRASweepMode	// 0x04B6
#define			StMeasFunc_PMC_UcCurrentSign_A	0x0001 + StMeasFunc_PMC_UcPrevSign_A	// 0x04B7
#define			StMeasFunc_PMC_UcCrossDetectA	0x0001 + StMeasFunc_PMC_UcCurrentSign_A	// 0x04B8
#define			StMeasFunc_PMC_UcPrevSign_B		0x0001 + StMeasFunc_PMC_UcCrossDetectA	// 0x04B9
#define			StMeasFunc_PMC_UcCurrentSign_B	0x0001 + StMeasFunc_PMC_UcPrevSign_B	// 0x04BA
#define			StMeasFunc_PMC_UcCrossDetectB	0x0001 + StMeasFunc_PMC_UcCurrentSign_B	// 0x04BB
#define			StMeasFunc_PMC_SiFsCountF		0x0004 + StMeasFunc_PMC_UcCrossDetectB	// 0x04BC
#define			StMeasFunc_PMC_SiFsCountR		0x0004 + StMeasFunc_PMC_SiFsCountF		// 0x04C0
#define			StMeasFunc_MFA_SiSampleNumA		0x0004 + StMeasFunc_PMC_SiFsCountR		// 0x04C4
#define			StMeasFunc_MFA_SiSampleMaxA		0x0004 + StMeasFunc_MFA_SiSampleNumA	// 0x04C8
#define			StMeasFunc_MFB_SiSampleNumB		0x0004 + StMeasFunc_MFA_SiSampleMaxA	// 0x04CC
#define			StMeasFunc_MFB_SiSampleMaxB		0x0004 + StMeasFunc_MFB_SiSampleNumB	// 0x04D0

//==============================================================================
//DMB
//==============================================================================
#define		SiVerNum						0x8000
#define		SiCalID							0x8004

#define		StCalibrationData				0x8010
				// Calibration.h  CalibrationData_Type
#define			StCaliData_UsCalibrationStatus	0x0000 + StCalibrationData
#define			StCaliData_SiHallMax_Before_X	0x0004 + StCaliData_UsCalibrationStatus
#define			StCaliData_SiHallMin_Before_X	0x0004 + StCaliData_SiHallMax_Before_X
#define			StCaliData_SiHallMax_After_X	0x0004 + StCaliData_SiHallMin_Before_X
#define			StCaliData_SiHallMin_After_X	0x0004 + StCaliData_SiHallMax_After_X
#define			StCaliData_SiHallMax_Before_Y	0x0004 + StCaliData_SiHallMin_After_X
#define			StCaliData_SiHallMin_Before_Y	0x0004 + StCaliData_SiHallMax_Before_Y
#define			StCaliData_SiHallMax_After_Y	0x0004 + StCaliData_SiHallMin_Before_Y
#define			StCaliData_SiHallMin_After_Y	0x0004 + StCaliData_SiHallMax_After_Y
#define			StCaliData_UiHallBias_X			0x0004 + StCaliData_SiHallMin_After_Y
#define			StCaliData_UiHallOffset_X		0x0004 + StCaliData_UiHallBias_X
#define			StCaliData_UiHallBias_Y			0x0004 + StCaliData_UiHallOffset_X
#define			StCaliData_UiHallOffset_Y		0x0004 + StCaliData_UiHallBias_Y
#define			StCaliData_SiLoopGain_X			0x0004 + StCaliData_UiHallOffset_Y
#define			StCaliData_SiLoopGain_Y			0x0004 + StCaliData_SiLoopGain_X
#define			StCaliData_SiLensCen_Offset_X	0x0004 + StCaliData_SiLoopGain_Y
#define			StCaliData_SiLensCen_Offset_Y	0x0004 + StCaliData_SiLensCen_Offset_X
#define			StCaliData_SiOtpCen_Offset_X	0x0004 + StCaliData_SiLensCen_Offset_Y
#define			StCaliData_SiOtpCen_Offset_Y	0x0004 + StCaliData_SiOtpCen_Offset_X
#define			StCaliData_SiGyroOffset_X		0x0004 + StCaliData_SiOtpCen_Offset_Y
#define			StCaliData_SiGyroOffset_Y		0x0004 + StCaliData_SiGyroOffset_X
#define			StCaliData_SiGyroGain_X			0x0004 + StCaliData_SiGyroOffset_Y
#define			StCaliData_SiGyroGain_Y			0x0004 + StCaliData_SiGyroGain_X
#define			StCaliData_UiHallBias_AF		0x0004 + StCaliData_SiGyroGain_Y
#define			StCaliData_UiHallOffset_AF		0x0004 + StCaliData_UiHallBias_AF
#define			StCaliData_SiLoopGain_AF		0x0004 + StCaliData_UiHallOffset_AF
#define			StCaliData_SiAD_Offset_AF		0x0004 + StCaliData_SiLoopGain_AF
#define			StCaliData_SiMagnification_AF	0x0004 + StCaliData_SiAD_Offset_AF
#define			StCaliData_SiHallMax_Before_AF	0x0004 + StCaliData_SiMagnification_AF
#define			StCaliData_SiHallMin_Before_AF	0x0004 + StCaliData_SiHallMax_Before_AF
#define			StCaliData_SiHallMax_After_AF	0x0004 + StCaliData_SiHallMin_Before_AF
#define			StCaliData_SiHallMin_After_AF	0x0004 + StCaliData_SiHallMax_After_AF

#define		HallFilterCoeffX				0x8090
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffX_HXIGAIN		0x0000 + HallFilterCoeffX
#define			HallFilterCoeffX_GYROXOUTGAIN	0x0004 + HallFilterCoeffX_HXIGAIN
#define			HallFilterCoeffX_HXOFFGAIN		0x0004 + HallFilterCoeffX_GYROXOUTGAIN

#define			HallFilterCoeffX_hxiab			0x0004 + HallFilterCoeffX_HXOFFGAIN
#define			HallFilterCoeffX_hxiac			0x0004 + HallFilterCoeffX_hxiab
#define			HallFilterCoeffX_hxiaa			0x0004 + HallFilterCoeffX_hxiac
#define			HallFilterCoeffX_hxibb			0x0004 + HallFilterCoeffX_hxiaa
#define			HallFilterCoeffX_hxibc			0x0004 + HallFilterCoeffX_hxibb
#define			HallFilterCoeffX_hxiba			0x0004 + HallFilterCoeffX_hxibc
#define			HallFilterCoeffX_hxdab			0x0004 + HallFilterCoeffX_hxiba
#define			HallFilterCoeffX_hxdac			0x0004 + HallFilterCoeffX_hxdab
#define			HallFilterCoeffX_hxdaa			0x0004 + HallFilterCoeffX_hxdac
#define			HallFilterCoeffX_hxdbb			0x0004 + HallFilterCoeffX_hxdaa
#define			HallFilterCoeffX_hxdbc			0x0004 + HallFilterCoeffX_hxdbb
#define			HallFilterCoeffX_hxdba			0x0004 + HallFilterCoeffX_hxdbc
#define			HallFilterCoeffX_hxdcc			0x0004 + HallFilterCoeffX_hxdba
#define			HallFilterCoeffX_hxdcb			0x0004 + HallFilterCoeffX_hxdcc
#define			HallFilterCoeffX_hxdca			0x0004 + HallFilterCoeffX_hxdcb
#define			HallFilterCoeffX_hxpgain0		0x0004 + HallFilterCoeffX_hxdca
#define			HallFilterCoeffX_hxigain0		0x0004 + HallFilterCoeffX_hxpgain0
#define			HallFilterCoeffX_hxdgain0		0x0004 + HallFilterCoeffX_hxigain0
#define			HallFilterCoeffX_hxpgain1		0x0004 + HallFilterCoeffX_hxdgain0
#define			HallFilterCoeffX_hxigain1		0x0004 + HallFilterCoeffX_hxpgain1
#define			HallFilterCoeffX_hxdgain1		0x0004 + HallFilterCoeffX_hxigain1
#define			HallFilterCoeffX_hxgain0		0x0004 + HallFilterCoeffX_hxdgain1
#define			HallFilterCoeffX_hxgain1		0x0004 + HallFilterCoeffX_hxgain0

#define			HallFilterCoeffX_hxsb			0x0004 + HallFilterCoeffX_hxgain1
#define			HallFilterCoeffX_hxsc			0x0004 + HallFilterCoeffX_hxsb
#define			HallFilterCoeffX_hxsa			0x0004 + HallFilterCoeffX_hxsc

#define			HallFilterCoeffX_hxob			0x0004 + HallFilterCoeffX_hxsa
#define			HallFilterCoeffX_hxoc			0x0004 + HallFilterCoeffX_hxob
#define			HallFilterCoeffX_hxod			0x0004 + HallFilterCoeffX_hxoc
#define			HallFilterCoeffX_hxoe			0x0004 + HallFilterCoeffX_hxod
#define			HallFilterCoeffX_hxoa			0x0004 + HallFilterCoeffX_hxoe
#define			HallFilterCoeffX_hxpb			0x0004 + HallFilterCoeffX_hxoa
#define			HallFilterCoeffX_hxpc			0x0004 + HallFilterCoeffX_hxpb
#define			HallFilterCoeffX_hxpd			0x0004 + HallFilterCoeffX_hxpc
#define			HallFilterCoeffX_hxpe			0x0004 + HallFilterCoeffX_hxpd
#define			HallFilterCoeffX_hxpa			0x0004 + HallFilterCoeffX_hxpe

#define		HallFilterCoeffY				0x812c
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffY_HYIGAIN		0x0000 + HallFilterCoeffY
#define			HallFilterCoeffY_GYROYOUTGAIN	0x0004 + HallFilterCoeffY_HYIGAIN
#define			HallFilterCoeffY_HYOFFGAIN		0x0004 + HallFilterCoeffY_GYROYOUTGAIN

#define			HallFilterCoeffY_hyiab			0x0004 + HallFilterCoeffY_HYOFFGAIN
#define			HallFilterCoeffY_hyiac			0x0004 + HallFilterCoeffY_hyiab
#define			HallFilterCoeffY_hyiaa			0x0004 + HallFilterCoeffY_hyiac
#define			HallFilterCoeffY_hyibb			0x0004 + HallFilterCoeffY_hyiaa
#define			HallFilterCoeffY_hyibc			0x0004 + HallFilterCoeffY_hyibb
#define			HallFilterCoeffY_hyiba			0x0004 + HallFilterCoeffY_hyibc
#define			HallFilterCoeffY_hydab			0x0004 + HallFilterCoeffY_hyiba
#define			HallFilterCoeffY_hydac			0x0004 + HallFilterCoeffY_hydab
#define			HallFilterCoeffY_hydaa			0x0004 + HallFilterCoeffY_hydac
#define			HallFilterCoeffY_hydbb			0x0004 + HallFilterCoeffY_hydaa
#define			HallFilterCoeffY_hydbc			0x0004 + HallFilterCoeffY_hydbb
#define			HallFilterCoeffY_hydba			0x0004 + HallFilterCoeffY_hydbc
#define			HallFilterCoeffY_hydcc			0x0004 + HallFilterCoeffY_hydba
#define			HallFilterCoeffY_hydcb			0x0004 + HallFilterCoeffY_hydcc
#define			HallFilterCoeffY_hydca			0x0004 + HallFilterCoeffY_hydcb
#define			HallFilterCoeffY_hypgain0		0x0004 + HallFilterCoeffY_hydca
#define			HallFilterCoeffY_hyigain0		0x0004 + HallFilterCoeffY_hypgain0
#define			HallFilterCoeffY_hydgain0		0x0004 + HallFilterCoeffY_hyigain0
#define			HallFilterCoeffY_hypgain1		0x0004 + HallFilterCoeffY_hydgain0
#define			HallFilterCoeffY_hyigain1		0x0004 + HallFilterCoeffY_hypgain1
#define			HallFilterCoeffY_hydgain1		0x0004 + HallFilterCoeffY_hyigain1
#define			HallFilterCoeffY_hygain0		0x0004 + HallFilterCoeffY_hydgain1
#define			HallFilterCoeffY_hygain1		0x0004 + HallFilterCoeffY_hygain0
#define			HallFilterCoeffY_hysb			0x0004 + HallFilterCoeffY_hygain1
#define			HallFilterCoeffY_hysc			0x0004 + HallFilterCoeffY_hysb
#define			HallFilterCoeffY_hysa			0x0004 + HallFilterCoeffY_hysc
#define			HallFilterCoeffY_hyob			0x0004 + HallFilterCoeffY_hysa
#define			HallFilterCoeffY_hyoc			0x0004 + HallFilterCoeffY_hyob
#define			HallFilterCoeffY_hyod			0x0004 + HallFilterCoeffY_hyoc
#define			HallFilterCoeffY_hyoe			0x0004 + HallFilterCoeffY_hyod
#define			HallFilterCoeffY_hyoa			0x0004 + HallFilterCoeffY_hyoe
#define			HallFilterCoeffY_hypb			0x0004 + HallFilterCoeffY_hyoa
#define			HallFilterCoeffY_hypc			0x0004 + HallFilterCoeffY_hypb
#define			HallFilterCoeffY_hypd			0x0004 + HallFilterCoeffY_hypc
#define			HallFilterCoeffY_hype			0x0004 + HallFilterCoeffY_hypd
#define			HallFilterCoeffY_hypa			0x0004 + HallFilterCoeffY_hype

#define		HallFilterLimitX				0x81c8
#define		HallFilterLimitY				0x81e0
#define		HallFilterShiftX				0x81f8
#define		HallFilterShiftY				0x81fe

#define		HF_MIXING						0x8214
#define			HF_hx45x						0x0000 + HF_MIXING			//0x008005E4 : HallMixingCoeff.hx45x
#define			HF_hx45y						0x0004 + HF_MIXING			//0x008005E8 : HallMixingCoeff.hx45y
#define			HF_hy45y						0x0008 + HF_MIXING			//0x008005EC : HallMixingCoeff.hy45y
#define			HF_hy45x						0x000C + HF_MIXING			//0x008005F0 : HallMixingCoeff.hy45x
#define			HF_ShiftX						0x0010 + HF_MIXING

#define		HAL_LN_CORRECT					0x8228
#define			HAL_LN_COEFAX					0x0000 + HAL_LN_CORRECT		//0x00800564 : HallLinearCorrAX.zone_coef[6]
#define			HAL_LN_COEFBX					0x000C + HAL_LN_COEFAX		//0x00800570 : HallLinearCorrBX.zone_coef[6]
#define			HAL_LN_ZONEX					0x000C + HAL_LN_COEFBX		//0x0080057C : HallLinearZoneX.zone_area[5]
#define			HAL_LN_COEFAY					0x000A + HAL_LN_ZONEX		//0x00800586 : HallLinearCorrAY.zone_coef[6]
#define			HAL_LN_COEFBY					0x000C + HAL_LN_COEFAY		//0x00800592 : HallLinearCorrBY.zone_coef[6]
#define			HAL_LN_ZONEY					0x000C + HAL_LN_COEFBY		//0x0080059E : HallLinearZoneY.zone_area[5]



#define		GyroFilterTableX				0x8270
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableX_gx45x			0x0000 + GyroFilterTableX
#define			GyroFilterTableX_gx45y			0x0004 + GyroFilterTableX_gx45x
#define			GyroFilterTableX_gxgyro			0x0004 + GyroFilterTableX_gx45y
#define			GyroFilterTableX_gxsengen		0x0004 + GyroFilterTableX_gxgyro
#define			GyroFilterTableX_gxl1b			0x0004 + GyroFilterTableX_gxsengen
#define			GyroFilterTableX_gxl1c			0x0004 + GyroFilterTableX_gxl1b
#define			GyroFilterTableX_gxl1a			0x0004 + GyroFilterTableX_gxl1c
#define			GyroFilterTableX_gxl2b			0x0004 + GyroFilterTableX_gxl1a
#define			GyroFilterTableX_gxl2c			0x0004 + GyroFilterTableX_gxl2b
#define			GyroFilterTableX_gxl2a			0x0004 + GyroFilterTableX_gxl2c
#define			GyroFilterTableX_gxigain		0x0004 + GyroFilterTableX_gxl2a
#define			GyroFilterTableX_gxh1b			0x0004 + GyroFilterTableX_gxigain
#define			GyroFilterTableX_gxh1c			0x0004 + GyroFilterTableX_gxh1b
#define			GyroFilterTableX_gxh1a			0x0004 + GyroFilterTableX_gxh1c
#define			GyroFilterTableX_gxk1b			0x0004 + GyroFilterTableX_gxh1a
#define			GyroFilterTableX_gxk1c			0x0004 + GyroFilterTableX_gxk1b
#define			GyroFilterTableX_gxk1a			0x0004 + GyroFilterTableX_gxk1c
#define			GyroFilterTableX_gxgain			0x0004 + GyroFilterTableX_gxk1a
#define			GyroFilterTableX_gxzoom			0x0004 + GyroFilterTableX_gxgain
#define			GyroFilterTableX_gxlenz			0x0004 + GyroFilterTableX_gxzoom
#define			GyroFilterTableX_gxt2b			0x0004 + GyroFilterTableX_gxlenz
#define			GyroFilterTableX_gxt2c			0x0004 + GyroFilterTableX_gxt2b
#define			GyroFilterTableX_gxt2a			0x0004 + GyroFilterTableX_gxt2c
#define			GyroFilterTableX_afzoom			0x0004 + GyroFilterTableX_gxt2a

#define		GyroFilterTableY				0x82D0
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableY_gy45y			0x0000 + GyroFilterTableY
#define			GyroFilterTableY_gy45x			0x0004 + GyroFilterTableY_gy45y
#define			GyroFilterTableY_gygyro			0x0004 + GyroFilterTableY_gy45x
#define			GyroFilterTableY_gysengen		0x0004 + GyroFilterTableY_gygyro
#define			GyroFilterTableY_gyl1b			0x0004 + GyroFilterTableY_gysengen
#define			GyroFilterTableY_gyl1c			0x0004 + GyroFilterTableY_gyl1b
#define			GyroFilterTableY_gyl1a			0x0004 + GyroFilterTableY_gyl1c
#define			GyroFilterTableY_gyl2b			0x0004 + GyroFilterTableY_gyl1a
#define			GyroFilterTableY_gyl2c			0x0004 + GyroFilterTableY_gyl2b
#define			GyroFilterTableY_gyl2a			0x0004 + GyroFilterTableY_gyl2c
#define			GyroFilterTableY_gyigain		0x0004 + GyroFilterTableY_gyl2a
#define			GyroFilterTableY_gyh1b			0x0004 + GyroFilterTableY_gyigain
#define			GyroFilterTableY_gyh1c			0x0004 + GyroFilterTableY_gyh1b
#define			GyroFilterTableY_gyh1a			0x0004 + GyroFilterTableY_gyh1c
#define			GyroFilterTableY_gyk1b			0x0004 + GyroFilterTableY_gyh1a
#define			GyroFilterTableY_gyk1c			0x0004 + GyroFilterTableY_gyk1b
#define			GyroFilterTableY_gyk1a			0x0004 + GyroFilterTableY_gyk1c
#define			GyroFilterTableY_gygain			0x0004 + GyroFilterTableY_gyk1a
#define			GyroFilterTableY_gyzoom			0x0004 + GyroFilterTableY_gygain
#define			GyroFilterTableY_gylenz			0x0004 + GyroFilterTableY_gyzoom
#define			GyroFilterTableY_gyt2b			0x0004 + GyroFilterTableY_gylenz
#define			GyroFilterTableY_gyt2c			0x0004 + GyroFilterTableY_gyt2b
#define			GyroFilterTableY_gyt2a			0x0004 + GyroFilterTableY_gyt2c
#define			GyroFilterTableY_afzoom			0x0004 + GyroFilterTableY_gyt2a

#define		GyroFilterShiftX				0x8338
				// GyroFilterCoeff.h  GF_Shift_t
#define			RG_GX2X4XF						0x0000 + GyroFilterShiftX
#define			RG_GX2X4XB						0x0001 + RG_GX2X4XF
#define			RG_GXOX							0x0002 + RG_GX2X4XB
#define			RG_GXAFZ						0x0003 + RG_GXOX

#define		GyroFilterShiftY				0x833C
				// GyroFilterCoeff.h  GF_Shift_t
#define			RG_GY2X4XF						0x0000 + GyroFilterShiftY
#define			RG_GY2X4XB						0x0001 + RG_GY2X4XF
#define			RG_GYOX							0x0002 + RG_GY2X4XB
#define			RG_GYAFZ						0x0003 + RG_GYOX

#define		MeasureFilterA_Coeff			0x8380
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			0x0000 + MeasureFilterA_Coeff
#define			MeasureFilterA_Coeff_c1			0x0004 + MeasureFilterA_Coeff_b1
#define			MeasureFilterA_Coeff_a1			0x0004 + MeasureFilterA_Coeff_c1
#define			MeasureFilterA_Coeff_b2			0x0004 + MeasureFilterA_Coeff_a1
#define			MeasureFilterA_Coeff_c2			0x0004 + MeasureFilterA_Coeff_b2
#define			MeasureFilterA_Coeff_a2			0x0004 + MeasureFilterA_Coeff_c2

#define		MeasureFilterB_Coeff			0x8398
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			0x0000 + MeasureFilterB_Coeff
#define			MeasureFilterB_Coeff_c1			0x0004 + MeasureFilterB_Coeff_b1
#define			MeasureFilterB_Coeff_a1			0x0004 + MeasureFilterB_Coeff_c1
#define			MeasureFilterB_Coeff_b2			0x0004 + MeasureFilterB_Coeff_a1
#define			MeasureFilterB_Coeff_c2			0x0004 + MeasureFilterB_Coeff_b2
#define			MeasureFilterB_Coeff_a2			0x0004 + MeasureFilterB_Coeff_c2

#define		OLAF_SETTING					0x85D8
				// OpenLoopAF.h	OLAF_SETTLING_t
#define			OLAF_Long_M_RRMD1					0x0000 + OLAF_SETTING
#define			OLAF_Long_M_FT						0x0002 + OLAF_SETTING
#define			OLAF_Long_I_RRMD1					0x0004 + OLAF_SETTING
#define			OLAF_Long_I_FT						0x0006 + OLAF_SETTING
#define			OLAF_Short_IIM_RRMD1				0x0008 + OLAF_SETTING
#define			OLAF_Short_IIM_FT					0x000A + OLAF_SETTING
#define			OLAF_Short_IMI_RRMD1				0x000C + OLAF_SETTING
#define			OLAF_Short_IMI_FT					0x000E + OLAF_SETTING
#define			OLAF_Short_MIM_RRMD1				0x0010 + OLAF_SETTING
#define			OLAF_Short_MIM_FT					0x0012 + OLAF_SETTING
#define			OLAF_Short_MMI_RRMD1				0x0014 + OLAF_SETTING
#define			OLAF_Short_MMI_FT					0x0016 + OLAF_SETTING

#define		MeasureFilterA_Temp				0x85F0
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Temp_b1			0x0000 + MeasureFilterA_Temp			// 0x85F4
#define			MeasureFilterA_Temp_c1			0x0004 + MeasureFilterA_Temp_b1			// 0x85F8
#define			MeasureFilterA_Temp_a1			0x0004 + MeasureFilterA_Temp_c1			// 0x85FC
#define			MeasureFilterA_Temp_b2			0x0004 + MeasureFilterA_Temp_a1			// 0x8600
#define			MeasureFilterA_Temp_c2			0x0004 + MeasureFilterA_Temp_b2			// 0x8604
#define			MeasureFilterA_Temp_a2			0x0004 + MeasureFilterA_Temp_c2			// 0x8608

#define		MeasureFilterB_Temp				0x8608
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Temp_b1			0x0000 + MeasureFilterB_Temp			// 0x860C
#define			MeasureFilterB_Temp_c1			0x0004 + MeasureFilterB_Temp_b1			// 0x8610
#define			MeasureFilterB_Temp_a1			0x0004 + MeasureFilterB_Temp_c1			// 0x8614
#define			MeasureFilterB_Temp_b2			0x0004 + MeasureFilterB_Temp_a1			// 0x8618
#define			MeasureFilterB_Temp_c2			0x0004 + MeasureFilterB_Temp_b2			// 0x861C
#define			MeasureFilterB_Temp_a2			0x0004 + MeasureFilterB_Temp_c2			// 0x8620

#define		GF_LimitX						0x8620
				// GyroFilterCoeff.h  GF_Limit_t
#define			GF_LimitX_H2LMT					0x0000 + GF_LimitX
#define			GF_LimitX_JLIMT					0x0004 + GF_LimitX_H2LMT
#define			GF_LimitX_HLIMT					0x0004 + GF_LimitX_JLIMT

#define		GF_LimitY						0x862C
				// GyroFilterCoeff.h  GF_Limit_t
#define			GF_LimitY_H2LMT					0x0000 + GF_LimitY
#define			GF_LimitY_JLIMT					0x0004 + GF_LimitY_H2LMT
#define			GF_LimitY_HLIMT					0x0004 + GF_LimitY_JLIMT


//==============================================================================
//IO
//==============================================================================
// System Control配置アドレス
#define 		SYSDSP_DSPDIV					0xD00014
#define 		SYSDSP_SOFTRES					0xD0006C
#define 		OSCRSEL							0xD00090	// OSC Frequency 1
#define 		OSCCURSEL						0xD00094	// OSC Frequency 2
#define 		SYSDSP_REMAP					0xD000AC
#define 		SYSDSP_CVER						0xD00100
//#define 		IOPLEVR							0xD00104	// IO port level read
// 簡易DAC出力電圧設定
#define			VGAVREF							0xD00280

#define 		ADDA_FSCNT							0xD01004
#define 		ADDA_FSCTRL							0xD01008
#define 		ADDA_ADDAINT						0xD0100C
#define 		ADDA_ADE							0xD01010
#define 		ADDA_ADAV							0xD01014
#define 		ADDA_ADORDER						0xD01018
#define 		ADDA_EXTEND							0xD0101C
#define 		ADDA_AD0O							0xD01020
#define 		ADDA_AD1O							0xD01024
#define 		ADDA_AD2O							0xD01028
#define 		ADDA_AD3O							0xD0102C

#define 		ADDA_DASELW							0xD01040
#define 		ADDA_DASU							0xD01044
#define 		ADDA_DAHD							0xD01048
#define 		ADDA_DASWAP							0xD0104C
#define 		ADDA_DASEL							0xD01050
#define 		ADDA_DAO							0xD01054

// PWM I/F配置アドレス
#define 		OISDRVFC5							0xD02100
#define 		OISDRVFC6							0xD02104
#define 		OISDRVFC7							0xD02108
#define 		OISDRVFC8							0xD0210C

#define			DRVCH1SEL							0xD02128
#define			DRVCH2SEL							0xD0212C

#define 		OISGAINAM							0xD02190
#define 		OISOFSTAM							0xD02194
#define 		OISGAINBM							0xD02198
#define 		OISOFSTBM							0xD0219C

#define 		AFDRVFC5							0xD02200
#define 		AFDRVFC6							0xD02204

#define 		AFGAINM								0xD02290
#define 		AFSOFSTM							0xD02294

#define FLASHROM_123F40		0xE07000	// Flash Memory I/F配置アドレス
#define 		FLASHROM_F40_RDATL					(FLASHROM_123F40 + 0x00)
#define 		FLASHROM_F40_RDATH					(FLASHROM_123F40 + 0x04)
#define 		FLASHROM_F40_WDATL					(FLASHROM_123F40 + 0x08)
#define 		FLASHROM_F40_WDATH					(FLASHROM_123F40 + 0x0C)
#define 		FLASHROM_F40_ADR					(FLASHROM_123F40 + 0x10)
#define 		FLASHROM_F40_ACSCNT					(FLASHROM_123F40 + 0x14)
#define 		FLASHROM_F40_CMD					(FLASHROM_123F40 + 0x18)
#define 		FLASHROM_F40_WPB					(FLASHROM_123F40 + 0x1C)
#define 		FLASHROM_F40_INT					(FLASHROM_123F40 + 0x20)
#define 		FLASHROM_F40_TRC					(FLASHROM_123F40 + 0x24)
#define 		FLASHROM_F40_TCFSH					(FLASHROM_123F40 + 0x28)
#define 		FLASHROM_F40_TCONFEN				(FLASHROM_123F40 + 0x2C)
#define 		FLASHROM_F40_TNVS					(FLASHROM_123F40 + 0x30)
#define 		FLASHROM_F40_TPGS					(FLASHROM_123F40 + 0x34)
#define 		FLASHROM_F40_TPROG					(FLASHROM_123F40 + 0x38)
#define 		FLASHROM_F40_TADSH					(FLASHROM_123F40 + 0x3C)
#define 		FLASHROM_F40_TRCVP					(FLASHROM_123F40 + 0x40)
#define 		FLASHROM_F40_TRCVS					(FLASHROM_123F40 + 0x44)
#define 		FLASHROM_F40_TRCVC					(FLASHROM_123F40 + 0x48)
#define 		FLASHROM_F40_TERASES				(FLASHROM_123F40 + 0x4C)
#define 		FLASHROM_F40_TERASEC				(FLASHROM_123F40 + 0x50)
#define 		FLASHROM_F40_TRW					(FLASHROM_123F40 + 0x54)

#define 		FLASHROM_F40_SPECIAL				(FLASHROM_123F40 + 0x5C)
#define 		FLASHROM_F40_FLASHTEST				(FLASHROM_123F40 + 0x60)
#define 		FLASHROM_F40_ERR_FLASH				(FLASHROM_123F40 + 0x68)
#define 		FLASHROM_F40_ALA					(FLASHROM_123F40 + 0x70)
#define 		FLASHROM_F40_ERR_ALA				(FLASHROM_123F40 + 0x74)

#define 		FLASHROM_F40_RSTB_FLA				(FLASHROM_123F40 + 0x4CC)
#define 		FLASHROM_F40_UNLK_CODE1				(FLASHROM_123F40 + 0x554)
#define 		FLASHROM_F40_CLK_FLAON				(FLASHROM_123F40 + 0x664)
#define 		FLASHROM_F40_UNLK_CODE2				(FLASHROM_123F40 + 0xAA8)
#define 		FLASHROM_F40_UNLK_CODE3				(FLASHROM_123F40 + 0xCCC)
#define 		FLASHROM_F40_RSTB_DEC				(FLASHROM_123F40 + 0xDB4)

