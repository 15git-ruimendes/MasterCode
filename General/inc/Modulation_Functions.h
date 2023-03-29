/**
 * @file Modulation_STM32.h
 * @author Rui Mendes
 * @brief 
 * @version 0.1
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdlib.h>
#include <math.h>

/**
 * @brief Class for sinusoidal modulation  
 * @note   
 * @retval None
 */
typedef struct
{
    int     MaxDC;
    int     MinDC;
    float*  OffSet;
    float   PI_2;
    int     PhaseNumber;
    float   Frequency;
    float   IncFrequency;
    float   Amplitude;
    float*  DutyCycles;
    int*    DC;
}Sin_Modulation;

int InitSinModulatio(int PHNumber,Sin_Modulation* SinMod);
int FreeSinModulation(Sin_Modulation* SinMod);
int Sinusoidal(Sin_Modulation* SinMod);


/**
 * @brief Class for SVPWM modulation  
 * @note   
 * @retval None
 */
typedef struct 
{
    int MaxDC;
    int MinDC;
    int     PhaseNumber;
    float   Frequency;
    float   IncFrequnecy;
    float   Amplitude;
    float   Angle;
    float   Alpha;
    float   Beta;
    float*  DutyCycles;
    int*    DC;
}SVM_Modulation;


int InitSVM_Modulation(int PHNumber,SVM_Modulation* SVMMod);
int FreeSVM_Modulation(SVM_Modulation* SVMMod);

int SVM_AlBet(SVM_Modulation* SVMMod);
int SVM_ModAngle(SVM_Modulation* SVMMod);


/**
 * @brief  Class for phase shift modulation
 * @note   
 * @retval None
 * todo Phase shift modulation for chopper operation.
 */
