/**
 * @file Modulation_STM32.c
 * @author Rui Mendes
 * @brief 
 * @version 0.1
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../inc/Modulation_Functions.h"

/**
 * @brief  Initialization of the duty cycle buffer with the size of the corresponding phases.
 * @note   
 * @param  PhaseNumber Number of PWM to modulate.
 * @retval None
 */
int InitSinModulatio(int PHNumber,Sin_Modulation* SinMod)
{
    SinMod->MaxDC = 65535;
    SinMod->MinDC = 0;
    SinMod->PI_2 = 6.283185307179586;
    SinMod->PhaseNumber     = PHNumber;
    SinMod->DutyCycles      = (float*)malloc(SinMod->PhaseNumber*sizeof(float));
    SinMod->OffSet          = (float*)malloc(SinMod->PhaseNumber*sizeof(float));
    SinMod->DC              = (int*)malloc(SinMod->PhaseNumber*sizeof(int));

    for (int i = 0; i < SinMod->PhaseNumber; i++)
    {
        SinMod->OffSet[i] = i* SinMod->PI_2/SinMod->PhaseNumber;
    }
    return 1;
}

int FreeSinModulation(Sin_Modulation* SinMod)
{
    free(SinMod->DC);
    free(SinMod->DutyCycles);
    free(SinMod->OffSet);
    return 1;
}

/**
 * @brief  Simple sinusoidal PWM where only amplitude, interruption time or SW frequency and number of phases is required.
 * @note   
 * @param  PhaseNumber Number of equaly phased out phases.
 * @param  Amplitude Amplitude or modulation coeeficient between 0-1.
 * @param  PhaseFrequency Base frequency of the output phases.
 * @param  SWFrequency Switching frequency or the freqeuency at which the function is called.
 * @retval No return value, but D_x values for duty cycle from 0-10000 are altered.
 */
int Sinusoidal(Sin_Modulation* SinMod)
{
    static float Time       =   0;
    float aux = 0;
    Time = (Time + 1/SinMod->IncFrequency > 1/SinMod->Frequency)?0:Time + 1/SinMod->IncFrequency;

    for (int i = 0; i < SinMod->PhaseNumber; i++)
    {
        aux = sin(SinMod->PI_2*Time*SinMod->Frequency + SinMod->OffSet[i]);
        SinMod->DutyCycles[i]  =   SinMod->Amplitude*aux;
        SinMod->DC[i]          =   (int)(SinMod->Amplitude*(0.5*(SinMod->MaxDC - SinMod->MinDC)*aux + 0.5*(SinMod->MaxDC + SinMod->MinDC)));
        SinMod->DC[i]          =   (SinMod->DC[i] > SinMod->MaxDC*0.99)?SinMod->MaxDC:SinMod->DC[i];
    }
}   

