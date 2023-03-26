/**
 * @file 0-CardHandler.h
 * @author Rui Mendes
 * @brief 
 * @version 0.1
 * @date 2023-03-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _CARD_HANDLER_
#define _CARD_HANDLER_

/*
 * Private Includes
 */
#include <stdint.h>
//#include "main.h"

/*
 * Private Definitions 
 */

#define MASTER_CODE

#define SAC_A_CODE
#define SAC_B_CODE
#define IIS_A_CODE
#define IIS_B_CODE
#define TAS_CODE
#define DIGC_CODE
#define COM_CODE

#define START
#define STOP
#define RECALL

/*
 * Private Structures
 */

/**
 * @brief  Generic Card value structure
 * @note   
 * @retval None
 */
typedef struct 
{
    uint32_t CAN_Code;
    
    uint32_t Last_RxData;

    uint32_t Last_TxData;

    uint32_t State;

    uint32_t Last_Events;    
}GenCard;

/**
 * @brief Master card, system backend handler  
 * @note   
 * @retval None
 */
typedef struct 
{
    GenCard SACA;
    GenCard SACB;

    GenCard IISA;
    GenCard IISB;

    GenCard TAS;
    GenCard DIGC;

}CardHandler;

/*
 * Private Functions
 */
void idBuilder();
void idParser();

/*
 * Public Functions
 */
void StartCard();
void StopCard();
void RecallCard();

void StartCard_T();
void StopCard_T();
void RecallCard_T();

void ParseTemperature();
void ParseSensors();

void SwitchKPC();
void SwitchMain();
void SwitchDC();

void SwitchOutputs();
void ParseInputs();

#endif