/**
* @brief    Acquisition functions, filters and transforms
* @file     Acquisition_Functions.h
* @author   Rui Mendes
* @date     26-09-2022
*/
#ifndef _ACQUISITION_H_
#define _ACQUISITION_H_

#include "Control_Functions.h"
#include <math.h>
#include <stdlib.h>
/*
* 
* GENERAL DEFINITIONS 
* 
*/
#ifndef _2_PI_
    #define _2_PI_ 6.283185307179586f
#endif 
#ifndef _PI_
    #define _PI_ 3.141592653589793f
#endif 

/**
 * 
 * TRANSFORMATION STRUTURE
 * 
*/
/**
 * @brief  Class used for the execution of the transforms
 * from ABC to AB and DQ spaces
 * @note   
 * @retval None
 */
typedef struct 
{
    float* ABC;
    float* ABG;
    float* DQ0;
    float  Rad_Angle;
}Transforms;

/**
 * 
 * TRANSFORM FUNCTIONS
 * 
*/

int   InitTransforms(Transforms* TRF);
int   FreeTransforms(Transforms* TRF);

float ABC_AlBet(Transforms* TRF);
float ABC_AlBet_PI(Transforms* TRF);
float AlBet_DQ0(Transforms* TRF);

float AlBet_ABC(Transforms* TRF);
float AlBet_ABC_PI(Transforms* TRF);
float DQ0_AlBet(Transforms* TRF);

float T_ABC_AlBet(float* ABC,float* ABG);
float T_ABC_AlBet_PI(float* ABC,float* ABG);
float T_AlBet_DQ0(float* ABG,float* DQ0,float  Rad_Angle);

float T_AlBet_ABC(float* ABG,float* ABC);
float T_AlBet_ABC_PI(float* ABG,float* ABC);
float T_DQ0_AlBet(float* DQ0,float* ABG,float  Rad_Angle);

/** * 
 * SIGNAL STRUCTURE
 * 
*/
typedef struct 
{
    float* FactorVector;
    float* PreviousVals;
    float OutputValue;
}FIR;

typedef struct 
{
    float* FactorVectorA;
    float* FactorVectorB;
    float* PreviousVals;
    float OutputValue;
}IIR;

typedef struct
{
    uint32_t Resettable;
    uint32_t SatFlag;

    float Out;
    float Out_N1;

    float MaxValue;
    float MinValue;

    float ResetValue_L;
    float ResetValue_H;
    float T;
}INTG;

typedef struct
{
    float Alpha;
    float Beta;

    float Alpha_N1;
    float Beta_N1;

    float wFF;
    float Ke;
    float T;
}SOGI;

typedef struct 
{
    float Ke;
    float Kp;
    float Ki;

    float T;

    float wFF;
    float wFF_1;
    float const_1;

    float wPLL;
    float o1;
    float oPLL;
    
    float Alpha_Adjust;
    float Beta_Adjust;

    INTG Integrator;
    SOGI SG;
    Transforms AlBetDQ;
    PI_Controller Controller;
}FFSOGI_PLL;

/**
 * 
 * SIGNAL FUNCTIONS
 * 
*/

void initFIR(FIR* Filter,float* Constant, uint32_t Order);
void initIIR(IIR* Filter, float* A_Constant,float* B_Constant,uint32_t Order);
void initINTG(INTG* Integrator,float PreSet,uint32_t Resettable,float MaxVal,float MinVal,float Reset_L,float Reset_H,float T);
void initSOGI(SOGI* SG,float wFF, float Ke,float T);
void initFFSOGI(FFSOGI_PLL* SPLL,float Ke,float T,float Kp,float Ki,float wPLL,float wFF);

void calcINTG(INTG* Integrator,float input);
void calcSOGI(SOGI* SG, float input);
void calcFFSOGI_PLL(FFSOGI_PLL* SPLL, float input);

void resetINTG(INTG* Integrator);

#endif
