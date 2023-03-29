/**
 * @file Acquisition_Functions.c
 * @author Rui Mendes
 * @brief 
 * @version 0.1
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Acquisition_Functions.h"

/**
 * 
 * SIGNAL FUNCTIONS
 * 
*/
void initFIR(FIR* Filter, float* Constant,uint32_t Order)
{

}

void initIIR(IIR* Filter, float* A_Constant,float* B_Constant,uint32_t Order)
{
    
}

/**
 * @brief  Initialization of the integrator function
 * @note   
 * @param  Integrator: Pointer to integrator structure
 * @param  PreSet: Initialization value
 * @param  Resettable: Resettable flag
 * @param  MaxVal: Maximum output value
 * @param  MinVal: Minium output value
 * @param  Reset_L: Value to reset to after passin min value
 * @param  Reset_H: Value to reset to after passing max value
 * @param  T: Sample Time 
 * @retval None
 */
void initINTG(INTG* Integrator,float PreSet,uint32_t Resettable,float MaxVal,float MinVal,float Reset_L,float Reset_H,float T)
{
    Integrator->Out = PreSet;
    Integrator->MaxValue = MaxVal;
    Integrator->MinValue = MinVal;
    Integrator->Resettable = Resettable;
    Integrator->SatFlag = 0;
    Integrator->ResetValue_H = Reset_H;
    Integrator->ResetValue_L = Reset_L;
    Integrator->T = T;
}

void initSOGI(SOGI* SG,float wFF,float Ke,float T)
{
    SG->wFF = wFF;
    SG->Ke = Ke;
    SG->T = T;
}

void initFFSOGI(FFSOGI_PLL* SPLL,float Ke,float T,float Kp,float Ki,float wPLL,float wFF)
{
    SPLL->Ke = Ke;
    SPLL->T = T;
    SPLL->Kp = Kp;
    SPLL->Ki = Ki;
    SPLL->wPLL = wPLL;
    SPLL->wFF = wFF;
    SPLL->wFF_1 = 1/wFF;

    SPLL->const_1 = 1/(SPLL->Ke*SPLL->wFF); 

    initINTG(&SPLL->Integrator,0,1,_2_PI_,0,_2_PI_,0,T);
    initSOGI(&SPLL->SG,wFF,Ke,T);
    PI_init(&SPLL->Controller,SPLL->Kp,SPLL->Ki,0,1000,-1000,T);
    InitTransforms(&SPLL->AlBetDQ);
}

/**
 * @brief Integral calculation with saturation and reset capabilities
 * @note   
 * @param  Integrator: Integrator pointer to structure
 * @param  input: Current input values
 * @retval None
 */
void calcINTG(INTG* Integrator,float input)
{
   static float AuxValue;
    Integrator->Out_N1 = Integrator->Out;

   AuxValue = Integrator->Out_N1 + Integrator->T * input;

    if (Integrator->Resettable)
    {
        Integrator->Out = (AuxValue > Integrator->MaxValue)?(Integrator->ResetValue_H + AuxValue - Integrator->MaxValue):AuxValue;
        //Integrator->Out = (AuxValue < Integrator->MinValue)?(Integrator->ResetValue_L + AuxValue - Integrator->MinValue):AuxValue;
    }
    else
    {
        Integrator->SatFlag = ((AuxValue > Integrator->MaxValue)||(AuxValue <Integrator->MinValue));
        Integrator->Out = (AuxValue > Integrator->MaxValue)?Integrator->MaxValue:AuxValue;
        Integrator->Out = (AuxValue < Integrator->MinValue)?Integrator->MinValue:AuxValue;
    }
}

/**
 * @brief  Calculation of the alpha beta components through SOGI algorithm
 * @note   Return values within the SOGI structure
 * @param  SG: Structure for SOGI phase detector 
 * @param  input: Input sinusoidal data
 * @retval None
 */
void calcSOGI(SOGI* SG, float input)
{
    SG->Alpha_N1 = SG->Alpha;
    SG->Beta_N1 = SG->Beta;

    SG->Beta = SG->Beta_N1 + (SG->wFF*SG->T)*SG->Alpha_N1;
    SG->Alpha = SG->Alpha_N1 + (SG->wFF*SG->T)*(SG->Ke*(input - SG->Alpha_N1)-SG->Beta);
}

/**
 * @brief  Calculation function to gather the angle of the grid
 * @note   Return values within the SOGI-PLL structure
 * @param  SPLL: SOGI_PLL Structure pointer
 * @param  input: 
 * @retval None
 */
void calcFFSOGI_PLL(FFSOGI_PLL* SPLL, float input)
{
    /*
    * Calculate SOGI
    */
    calcSOGI(&SPLL->SG,input);

   /*
   * Adjust Vbeta
   */
    SPLL->Alpha_Adjust = SPLL->SG.Alpha;
    SPLL->Beta_Adjust = SPLL->SG.Beta;//*SPLL->wPLL*SPLL->wFF_1;
    SPLL->AlBetDQ.ABG[0] = SPLL->Alpha_Adjust;
    SPLL->AlBetDQ.ABG[1] = SPLL->Beta_Adjust;

    /*
    * Perform Park Transform
    */
    SPLL->AlBetDQ.Rad_Angle = SPLL->o1;
    AlBet_DQ0(&SPLL->AlBetDQ);

    /*
    * Pass through PI value
    */
    PI_Calculate(&SPLL->Controller,-SPLL->AlBetDQ.DQ0[1]);

    /*
    * Compensate for wFF
    */
    SPLL->wPLL = SPLL->Controller.Result + SPLL->wFF;

    /*
    * Integrate frequency for angle o1
    */
    calcINTG(&SPLL->Integrator,SPLL->wPLL);
    SPLL->o1 = SPLL->Integrator.Out;

    /*
    * Adjust angle to obtain oPLL
    */
    float Correction = (SPLL->wPLL*SPLL->wPLL - SPLL->wFF*SPLL->wFF)*SPLL->const_1/SPLL->wPLL;
    
    /**
     * ! TO-DO: Check for saturation?  
    */
    SPLL->oPLL = SPLL->o1 + Correction; 
}



/**
 *
 * TRANSFORMATION FUNCTIONS
 * 
*/
int   InitTransforms(Transforms* TRF)
{
    TRF->ABC = (float*)malloc(3*sizeof(float));
    TRF->ABG = (float*)malloc(3*sizeof(float));
    TRF->DQ0 = (float*)malloc(3*sizeof(float));
    return 1;
}
int   FreeTransforms(Transforms* TRF)
{
    free(TRF->ABC);
    free(TRF->ABG);
    free(TRF->DQ0);
    return 1;
}

/**
 * @brief  ABC to Alpha-Beta-Gama transformation
 * @note   
 * @retval 
 */
float ABC_AlBet(Transforms* TRF)
{
    TRF->ABG[0] = 0.666666666666667*TRF->ABC[0] - 0.333333333333333*TRF->ABC[1] - 0.333333333333333*TRF->ABC[2];
    TRF->ABG[1] = 0.577350269189626*TRF->ABC[1] - 0.577350269189626*TRF->ABC[2];
    TRF->ABG[2] = 0.333333333333333*TRF->ABC[0] + 0.333333333333333*TRF->ABC[1] + 0.333333333333333*TRF->ABC[2];
    return 1;
}
/**
 * @brief  ABC to Alpha-Beta-Gama transformation Power invariant
 * @note   
 * @retval 
 */
float ABC_AlBet_PI(Transforms* TRF)
{
    TRF->ABG[0] = 0.816496580927726*TRF->ABC[0] - 0.408248290463863*TRF->ABC[1] - 0.408248290463863*TRF->ABC[2];
    TRF->ABG[1] = 0.707106781186547*TRF->ABC[1] - 0.707106781186547*TRF->ABC[2];
    TRF->ABG[2] = 0.577350269189626*TRF->ABC[0] + 0.577350269189626*TRF->ABC[1] + 0.577350269189626*TRF->ABC[2];
    return 1;
}
/**
 * @brief  Alpha-Beta-Gama to DQ0 transformation
 * @note   
 * @retval 
 */
float AlBet_DQ0(Transforms* TRF)
{
    TRF->DQ0[0] = cos(TRF->Rad_Angle)*TRF->ABG[0] + sin(TRF->Rad_Angle)*TRF->ABG[1];
    TRF->DQ0[1] = -sin(TRF->Rad_Angle)*TRF->ABG[0] + cos(TRF->Rad_Angle)*TRF->ABG[1];
    TRF->DQ0[2] = TRF->ABG[2];
    return 1;
}
/**
 * @brief  Alpha-Beta-Gama to ABC transformation
 * @note   
 * @retval 
 */
float AlBet_ABC(Transforms* TRF)
{
    TRF->ABC[0] = 1.000000000000000*TRF->ABG[0] + 1.000000000000000*TRF->ABG[2];
    TRF->ABC[1] = -0.50000000000000*TRF->ABG[0] + 0.866025403784439*TRF->ABG[1] + 1.000000000000000*TRF->ABG[2];
    TRF->ABC[2] = -0.50000000000000*TRF->ABG[0] - 0.866025403784439*TRF->ABG[1] + 1.000000000000000*TRF->ABG[2];
    return 1;
}
/**
 * @brief  Alpha-Beta-Gama to ABC transformation Power invariant
 * @note   
 * @retval 
 */
float AlBet_ABC_PI(Transforms* TRF)
{
    TRF->ABC[0] = 0.816496580927726*TRF->ABG[0] + 0.577350269189626*TRF->ABG[2];
    TRF->ABC[1] = -0.40824829046386*TRF->ABG[0] + 0.707106781186547*TRF->ABG[1] + 0.577350269189626*TRF->ABG[2];
    TRF->ABC[2] = -0.40824829046386*TRF->ABG[0] - 0.707106781186547*TRF->ABG[1] + 0.577350269189626*TRF->ABG[2];
    return 1;
}
/**
 * @brief  DQ0 to Alpha-Beta-Gama transformation
 * @note   
 * @retval 
 */
float DQ0_AlBet(Transforms* TRF)
{
    TRF->ABG[0] = cos(TRF->Rad_Angle)*TRF->DQ0[0] - sin(TRF->Rad_Angle)*TRF->DQ0[1];
    TRF->ABG[1] = sin(TRF->Rad_Angle)*TRF->DQ0[0] + cos(TRF->Rad_Angle)*TRF->DQ0[1];
    TRF->ABG[2] = TRF->DQ0[2];
    return 1;
}


/**
 * @brief  ABC to Alpha-Beta-Gama transformation
 * @note   
 * @param  ABC: Float vector with ABC values
 * @param  ABG: Return float vector with ABG values 
 * @retval 
 */
float T_ABC_AlBet(float* ABC,float* ABG)
{
    if (sizeof(ABC)*0.25 > 2.99 && sizeof(ABG)*0.25 > 2.99 )
    {
        ABG[0] = 0.666666666666667*ABC[0] - 0.333333333333333*ABC[1] - 0.333333333333333*ABC[2];
        ABG[1] = 0.577350269189626*ABC[1] - 0.577350269189626*ABC[2];
        ABG[2] = 0.333333333333333*ABC[0] + 0.333333333333333*ABC[1] + 0.333333333333333*ABC[2];
        return 1;
    }
    else
    {
        return -1;
    }
}
/**
 * @brief  ABC to Alpha-Beta-Gama transformation power invariant
 * @note   
 * @param  ABC: Float vector with ABC values
 * @param  ABG: Return float vector with ABG values 
 * @retval 
 */
float T_ABC_AlBet_PI(float* ABC,float* ABG)
{
    if (sizeof(ABC)*0.25 > 2.99 && sizeof(ABG)*0.25 > 2.99 )
    {
        ABG[0] = 0.816496580927726*ABC[0] - 0.408248290463863*ABC[1] - 0.408248290463863*ABC[2];
        ABG[1] = 0.707106781186547*ABC[1] - 0.707106781186547*ABC[2];
        ABG[2] = 0.577350269189626*ABC[0] + 0.577350269189626*ABC[1] + 0.577350269189626*ABC[2];
        return 1;
    }
    else
    {
        return -1;
    }
}
/**
 * @brief  Alpha-Beta-Gama to DQ0 transformation
 * @note   
 * @param  ABG: Float vector with ABG values 
 * @param  DQ0: Return float vector with dq0 values
 * @param  Rad_Angle: Angle of the reference frame
 * @retval 
 */
float T_AlBet_DQ0(float* ABG,float* DQ0,float  Rad_Angle)
{
    if (sizeof(ABG)*0.25 > 2.99 && sizeof(DQ0)*0.25 > 2.99 )
    {
        DQ0[0] = cos(Rad_Angle)*ABG[0] + sin(Rad_Angle)*ABG[1];
        DQ0[1] = -sin(Rad_Angle)*ABG[0] + cos(Rad_Angle)*ABG[1];
        DQ0[2] = ABG[2];
        return 1;
    }
    else
    {
        return -1;
    }
}
/**
 * @brief  Alpha-Beta-Gama to ABC transformation
 * @note   
 * @param  ABC: Float vector with ABC values
 * @param  ABG: Return float vector with ABG values  
 * @retval 
 */
float T_AlBet_ABC(float* ABG,float* ABC)
{
    if (sizeof(ABC)*0.25 > 2.99 && sizeof(ABG)*0.25 > 2.99 )
    {
        ABC[0] = 1.000000000000000*ABG[0] + 1.000000000000000*ABG[2];
        ABC[1] = -0.50000000000000*ABG[0] + 0.866025403784439*ABG[1] + 1.000000000000000*ABG[2];
        ABC[2] = -0.50000000000000*ABG[0] - 0.866025403784439*ABG[1] + 1.000000000000000*ABG[2];
        return 1;
    }
    else
    {
        return -1;
    }
}
/**
 * @brief  Alpha-Beta-Gama to ABC transformation
 * @note   
 * @param  ABG: Float vector with ABG values
 * @param  ABC: Return float vector with ABC values  
 * @retval 
 */
float T_AlBet_ABC_PI(float* ABG,float* ABC)
{
    if (sizeof(ABC)*0.25 > 2.99 && sizeof(ABG)*0.25 > 2.99 )
    {
        ABC[0] = 0.816496580927726*ABG[0] + 0.577350269189626*ABG[2];
        ABC[1] = -0.40824829046386*ABG[0] + 0.707106781186547*ABG[1] + 0.577350269189626*ABG[2];
        ABC[2] = -0.40824829046386*ABG[0] - 0.707106781186547*ABG[1] + 0.577350269189626*ABG[2];
        return 1;
    }
    else
    {
        return -1;
    }
}
/**
 * @brief  DQ0 to Alpha-Beta-Gama transformation
 * @note   
 * @param  DQ0: Float vector with dq0 values 
 * @param  ABG: Return float vector with ABG values
 * @param  Rad_Angle: Angle of the reference frame
 * @retval 
 */
float T_DQ0_AlBet(float* DQ0,float* ABG,float  Rad_Angle)
{
    if (sizeof(DQ0)*0.25 > 2.99 && sizeof(ABG)*0.25 > 2.99 )
    {
        ABG[0] = cos(Rad_Angle)*DQ0[0] - sin(Rad_Angle)*DQ0[1];
        ABG[1] = sin(Rad_Angle)*DQ0[0] + cos(Rad_Angle)*DQ0[1];
        ABG[2] = DQ0[2];
        return 1;
    }
    else
    {
        return -1;
    }
}
