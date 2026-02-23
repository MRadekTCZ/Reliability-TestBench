//#############################################################################
// FILE    : hvp045a_io.h.h
// TITLE   : Header file for demoboard i/o assignments
// Version : 1.0
//
//  Group         : C2000
//  Target Family : F2837x
//  Created on    : Jun 17, 2020
//  Author        : Han Zhang
//#############################################################################
// $TI Release: C2000 FCL SFRA $
// $Release Date: 05/2020 $
// $Copyright: Copyright (C) 2013-2018 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################


#ifndef _HVP045A_IO_H_
#define _HVP045A_IO_H_

/*****************************************************************************/
// Digital and Analog Pin assignments
/*****************************************************************************/

// ****************************************************************************
// ****************************************************************************
// Digital GPIO assignments
// ****************************************************************************
// ****************************************************************************
#define  PHASE_U_PWM_BASE          EPWM1_BASE

#define  PHASE_UH_GPIO             0
#define  PHASE_UH_GPIO_PWM_CFG     GPIO_0_EPWM1A
#define  PHASE_UH_GPIO_GPIO_CFG    GPIO_0_GPIO0

#define  PHASE_UL_GPIO             1
#define  PHASE_UL_GPIO_PWM_CFG     GPIO_1_EPWM1B
#define  PHASE_UL_GPIO_GPIO_CFG    GPIO_1_GPIO1
//===============================================
#define  PHASE_V_PWM_BASE          EPWM2_BASE

#define  PHASE_VH_GPIO             2
#define  PHASE_VH_GPIO_PWM_CFG     GPIO_2_EPWM2A
#define  PHASE_VH_GPIO_GPIO_CFG    GPIO_2_GPIO2

#define  PHASE_VL_GPIO             3
#define  PHASE_VL_GPIO_PWM_CFG     GPIO_3_EPWM2B
#define  PHASE_VL_GPIO_GPIO_CFG    GPIO_3_GPIO3
//===============================================
#define  PHASE_W_PWM_BASE          EPWM3_BASE

#define  PHASE_WH_GPIO             4
#define  PHASE_WH_GPIO_PWM_CFG     GPIO_4_EPWM3A
#define  PHASE_WH_GPIO_GPIO_CFG    GPIO_4_GPIO4

#define  PHASE_WL_GPIO             5
#define  PHASE_WL_GPIO_PWM_CFG     GPIO_5_EPWM3B
#define  PHASE_WL_GPIO_GPIO_CFG    GPIO_5_GPIO5

#define  GD_SPI_BASE             SPIA_BASE

#define  GD_SPISIMO_GPIO         58
#define  GD_SPISIMO_GPIO_CFG     GPIO_58_SPISIMOA

#define  GD_SPISOMI_GPIO         59
#define  GD_SPISOMI_GPIO_CFG     GPIO_59_SPISOMIA

#define  GD_SPICLK_GPIO          60
#define  GD_SPICLK_GPIO_CFG      GPIO_60_SPICLKA

#define  GD_SPISTE_GPIO          61
#define  GD_SPISTE_GPIO_CFG      GPIO_61_SPISTEA

//=========================================================
#define  LED_GPIO               31
#define  LED_GPIO_CFG           GPIO_31_GPIO31

// UCC5870 IOs
#define  ASC_EN_GPIO        86
#define  ASC_EN_GPIO_CFG    GPIO_86_GPIO86

#define  ASC_L_GPIO         88
#define  ASC_L_GPIO_CFG     GPIO_88_GPIO88

#define  ASC_H_GPIO         90
#define  ASC_H_GPIO_CFG     GPIO_90_GPIO90

#define  nFLT2L_GPIO        97
#define  nFLT2L_GPIO_CFG    GPIO_97_GPIO97

#define  nFLT1L_GPIO        52
#define  nFLT1L_GPIO_CFG    GPIO_52_GPIO52

#define  nFLT2H_GPIO        22
#define  nFLT2H_GPIO_CFG    GPIO_22_GPIO22

#define  nFLT1H_GPIO        67
#define  nFLT1H_GPIO_CFG    GPIO_67_GPIO67

//=============================================================================
#endif /* _HVP045A_IO_H_ */
