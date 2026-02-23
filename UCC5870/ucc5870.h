//#############################################################################
// FILE    : UCC5870.h
// TITLE   : Header file for UCC5870 interface modules
// Version : 1.0
//
//  Group         : C2000
//  Target Family : F2837x
//  Created on    : Jun 3, 2020
//  Author        : Ramesh Ramamoorthy
//#############################################################################
// $TI Release: C2000 FCL SFRA $
// $Release Date: 05/2020 $
// $Copyright: Copyright (C) 2013-2018 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################


#ifndef _UCC5870_H_
#define _UCC5870_H_

#include "device.h"
#include "ucc5870_regs.h"

//
// enumerated variables
//
typedef enum {
    ALL_GOOD      = 0,
    INIT_FAULT    = 1,
    STATUS_FAULT  = 2,
    PRI_RDY_FAULT = 3,
    SEC_RDY_FAULT = 4
} UCC5870_Status_e;

/*****************************************************************************/
// function prototypes
/*****************************************************************************/
#ifndef _UCC5870
extern
#endif
uint16_t  diagnose_UCC5870(uint16_t i);

#ifndef _UCC5870
extern
#endif
void     clearFaultsUCC5870(void);

#ifndef _UCC5870
extern
#endif
UCC5870_Status_e inverterDiagnostics(void);

#ifndef _UCC5870
extern
#endif
void     Init_UCC5870_Regs(void);

#ifndef _UCC5870
extern
#endif
UCC5870_Status_e  Init_UCC5870();


// local prototypes for use within ucc5870.c
void       cmdUCC5870 (uint16_t cmd);
uint16_t   readRegUCC5870 (uint16_t CA, uint16_t RA);
void       writeRegUCC5870(uint16_t CA, uint16_t RA, uint16_t  data);
uint16_t   writeVerifyRegUCC5870 (uint16_t CA, uint16_t RA, uint16_t  data);
uint16_t   writeVerify_UCC5870(uint16_t i);

//=============================================================================
#endif /* _UCC5870_H_ */
