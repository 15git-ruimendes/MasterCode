/**
 * @file Finite_State_Machines_XMC.c
 * @author Rui Mendes
 * @brief 
 * @version 0.1
 * @date 2022-09-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../inc/Finite_State_Machines_XMC.h"

/**
 * @brief  Typification of a FSM transition function including regularlly used expressions.
 * @note   Requires the declaration of FSM vector globally.
 * @retval None
 */
void CalculateTransitions(void)
{
    /* General analysis of FSMs */
    // Input Finite State Machine
    FSM[INPUT].State    = (FSM[INPUT].State == 0      &&   FSM[MASTER].State == 0 && IN_VOLT > IN_VOLT_MIN && IN_VOLT < IN_VOLT_MAX)?1:FSM[INPUT].State;
    FSM[INPUT].State    = ( FSM[MASTER].State != 0    &&   (IN_VOLT < IN_VOLT_MIN || IN_VOLT > IN_VOLT_MAX))?500:FSM[INPUT].State;

    // Output Finite State Machine
    FSM[OUTPUT].State   = (FSM[OUTPUT].State == 0     &&   FSM[MASTER].State == 1 && OUT_VOLT > PRE_CHRG_MIN && OUT_VOLT < PRE_CHRG_MAX)?1:FSM[OUTPUT].State;
    FSM[OUTPUT].State   = (FSM[MASTER].State != 1     &&   (OUT_VOLT < OUT_MIN || OUT_VOLT > OUT_MAX))?400:FSM[INPUT].State;

    // Contorl Finite State Machine
    FSM[CONTROL].State  = (FSM[CONTROL].State == 0    &&      FSM[MASTER].State == 2)                                               ?1:FSM[CONTROL].State;
    // Integral Sat State
    //FSM[CONTROL].State  = (FSM[CONTROL].State == 1    &&      Controller.IntSat)                                                    ?2:FSM[CONTROL].State;
    //FSM[CONTROL].State  = (FSM[CONTROL].State == 2    &&      !Controller.IntSat)                                                   ?1:FSM[CONTROL].State;
    // Block State
    FSM[CONTROL].State  = (FSM[CONTROL].State == 1   ||      FSM[CONTROL].State == 2 /*&& Controller.Fail*/)                              ?300:FSM[CONTROL].State;

    // PreCharge Finite State Machine
    FSM[PRE_CHRG].State = (FSM[PRE_CHRG].State == 0   &&      FSM[MASTER].State == 1)                                               ?1:FSM[PRE_CHRG].State;
    FSM[PRE_CHRG].State = (FSM[PRE_CHRG].State == 1   &&      FSM[OUTPUT].State == 1 && FSM[PRE_CHRG]._TimeOn > T_PRE_CHR)          ?2:FSM[PRE_CHRG].State;
    // Block State
    FSM[PRE_CHRG].State = (FSM[PRE_CHRG].State == 1   &&      FSM[OUTPUT].State == 400)                                             ?200:FSM[PRE_CHRG].State;

    // Master Finite State Machine 
    FSM[MASTER].State   = (FSM[MASTER].State == 0     &&      FSM[INPUT].State == 1 && Enable)                                      ?1:FSM[MASTER].State;
    FSM[MASTER].State   = (FSM[MASTER].State == 1     &&      FSM[PRE_CHRG].State == 2)                                             ?2:FSM[MASTER].State;
    FSM[MASTER].State   = (FSM[MASTER].State == 2     &&      FSM[OUTPUT].State == 1)                                               ?3:FSM[MASTER].State;
    FSM[MASTER].State   = (FSM[MASTER].State == 3     &&      FSM[CONTROL].State == 3)                                              ?4:FSM[MASTER].State;
    FSM[MASTER].State   = (FSM[MASTER].State == 4     &&      FSM[CONTROL].State == 2)                                              ?3:FSM[MASTER].State;
    // Block State
    FSM[MASTER].State   = (ANY_FAIL)?100:FSM[MASTER].State;
    // Reset State
    FSM[MASTER].State   = (FSM[MASTER].State == 100   &&      FSM[MASTER]._TimeOn > T_RESET_FAIL && !ANY_FAIL)                        ?0:FSM[MASTER].State;
    FSM[PRE_CHRG].State = (FSM[MASTER].State == 100   &&      FSM[MASTER]._TimeOn > T_RESET_FAIL && !ANY_FAIL)                        ?0:FSM[PRE_CHRG].State;
    FSM[CONTROL].State  = (FSM[MASTER].State == 100   &&      FSM[MASTER]._TimeOn > T_RESET_FAIL && !ANY_FAIL)                        ?0:FSM[CONTROL].State;
    FSM[INPUT].State    = (FSM[MASTER].State == 100   &&      FSM[MASTER]._TimeOn > T_RESET_FAIL && !ANY_FAIL)                        ?0:FSM[INPUT].State;

    // Outputs

    Kpc                = (FSM[PRE_CHRG].State == 1)?1:0;
    Kmain              = (FSM[PRE_CHRG].State == 2)?1:0;
    Kpc                = (FSM[MASTER].State   == 100)?0:Kpc;
    Kmain              = (FSM[MASTER].State   == 100)?0:Kmain;
    KShunt1            = (FSM[MASTER].State   == 100)?0:KShunt1;
    KShunt2            = (FSM[MASTER].State   == 100)?0:KShunt2;

}