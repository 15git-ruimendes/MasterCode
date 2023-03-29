#include "../inc/Control_Functions.h"

/**
 * @brief Implelentation of a discrete PID controller
 * 
 * @param In Input value.
 * @param Ref Reference value.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param Ts Execution time.
 * @param PrevError1 Previous cycle error pointer (Automatically updated).
 * @param PrevError2 Two cycles late error pointer (Automatically updated).
 * @param MaxLim Maximum output limit (Anti-Windup).
 * @param MinLim Minimum output limit (Anti-Windup).
 * @param Output Pointer to output value to be used in the next iteration (DO NOT CHANGE).
 * @return float Mirror of output value.
 */
float Pid_Controller(float In,float Ref,float Kp,float Ki,float Kd,float Ts,float* PrevError1,float* PrevError2,float MaxLim,float MinLim,float* Output)
{
    float Error     = Ref - In;
    float Td        = 1/Ts;
    float TestOut   = *Output + Error * (Kp + Ki*Ts + Kd*Td) - *PrevError1 * (Kp + 2*Ki*Ts) + *PrevError2 * Kd * Td;

    TestOut         = (TestOut > MaxLim)?MaxLim:TestOut;
    TestOut         = (TestOut < MinLim)?MinLim:TestOut;

    *PrevError2  = *PrevError1;
    *PrevError1  = Error;
    *Output     = TestOut;
    return TestOut;
}

/**
 * @brief Implelentation of a discrete PI controller
 * 
 * @param In Input value.
 * @param Ref Reference value.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Ts Execution time.
 * @param PrevError Previous cycle error pointer (Automatically updated).
 * @param MaxLim Maximum output limit (Anti-Windup).
 * @param MinLim Minimum output limit (Anti-Windup).
 * @param Output Pointer to output value to be used in the next iteration (DO NOT CHANGE).
 * @return float Mirror of output value.
 */
float Pi_Controller(float In,float Ref, float Kp,float Ki,float Ts,float* PrevError,float MaxLim,float MinLim,float* Output)
{
    float Error     = Ref - In;

    float TestOut   = *Output + Error * (Kp + Ki*Ts) - *PrevError * Kp;

    TestOut         = (TestOut > MaxLim)?MaxLim:TestOut;
    TestOut         = (TestOut < MinLim)?MinLim:TestOut;

    *PrevError  = Error;
    *Output          = TestOut;
    return TestOut;
}

/**
 * @brief Implelentation of a discrete P controller
 * 
 * @param In Input value.
 * @param Ref Reference value.
 * @param Kp Proportional gain.
 * @param MaxLim Maximum output limit (Anti-Windup).
 * @param MinLim Minimum output limit (Anti-Windup).
 * @param Output Pointer to output value to be used in the next iteration (DO NOT CHANGE).
 * @return float Mirror of output value. 
 */
float P_Controller(float In,float Ref,float Kp,float MaxLim,float MinLim,float* Output)
{
    float Error     = Ref - In;
    float TestOut   = Error * Kp;

    TestOut         = (TestOut > MaxLim)?MaxLim:TestOut;
    TestOut         = (TestOut < MinLim)?MinLim:TestOut;

    *Output          = TestOut;
    return TestOut;
}

/**
 * @brief  Structure initialization P.
 * 
 * @param  Controller Pointer to P_Contorller instance.
 * @param  Kp Proportional gain.
 * @param  Reference Reference target value.
 * @param  MaxLim Maximum limit of output value.
 * @param  MinLim Minimum limit of output value.
 * @retval None
 */
void P_init(Prop_Controller* Controller, float Kp, float Reference, float MaxLim,float MinLim)
{
    Controller->Kp = Kp;
    Controller->Reference = Reference;
    Controller->MaxLim = MaxLim;
    Controller->MinLim = MinLim;

    Controller->Result = 0;
}

/**
 * @brief  Structure initialization PI.
 * 
 * @param  Controller Pointer to PI_Contorller instance.
 * @param  Kp Proportional gain.
 * @param  Ki Integrator gain.
 * @param  Reference Reference target value.
 * @param  MaxLim Maximum limit of output value.
 * @param  MinLim Minimum limit of output value.
 * @param  Ts Calculation interval of the integrator
 * @retval None
 */
void PI_init(PI_Controller* Controller, float Kp, float Ki, float Reference, float MaxLim,float MinLim,float Ts)
{
    Controller->Kp = Kp;
    Controller->Ki = Ki;
    Controller->Reference = Reference;
    Controller->MaxLim = MaxLim;
    Controller->MinLim = MinLim;

    Controller->Ts = Ts;

    Controller->Result = 0;
    Controller->IntegralSat = 0;
    Controller->PreviousError = 0;
}

/**
 * @brief  Structure initialization PID.
 * 
 * @param  Controller Pointer to PID_Contorller instance.
 * @param  Kp Proportional gain.
 * @param  Ki Integrator gain.
 * @param  Kd Derivative gain.
 * @param  Reference Reference target value.
 * @param  MaxLim Maximum limit of output value.
 * @param  MinLim Minimum limit of output value.
 * @param  Ts Calculation interval of the integrator
 * @retval None
 */
void PID_init(PID_Controller* Controller, float Kp, float Ki, float Kd, float Reference, float MaxLim,float MinLim, float Ts)
{
    Controller->Kp = Kp;
    Controller->Ki = Ki;
    Controller->Kd = Kd;
    Controller->Reference = Reference;
    Controller->MaxLim = MaxLim;
    Controller->MinLim = MinLim;

    Controller->Ts = Ts;
    Controller->Td = 1/Ts;

    Controller->Result = 0;
    Controller->IntegralSat = 0;
    Controller->PreviousError1 = 0;
    Controller->PreviousError2 = 0;
}

/**
 * @brief  Reset of output and integrator values
 * @note   
 * @param  Controller Pointer to PI controller
 * @retval New output value
 */
int PI_Reset(PI_Controller* Controller)
{
    Controller->Result = 0;
    Controller->PreviousError = 0;
    return 0;
}

/**
 * @brief  Reset of output and integrator values
 * @note   
 * @param  Controller Pointer to PID controller
 * @retval New output value
 */
int PID_Reset(PID_Controller* Controller)
{
    Controller->Result = 0;
    Controller->PreviousError1 = 0;
    Controller->PreviousError2 = 0;
    return 0;
}

/**
 * @brief  Calculation of proportional controller
 * @note   
 * @param  Controller P Controller pointer
 * @param  input Input value to be update control output
 * @retval 
 */
float P_Calculate(Prop_Controller* Controller,float input)
{
    float Error     = Controller->Reference - input;
    float TestOut   = Error * Controller->Kp;

    TestOut         = (TestOut > Controller->MaxLim)?Controller->MaxLim:TestOut;
    TestOut         = (TestOut < Controller->MinLim)?Controller->MinLim:TestOut;

    Controller->Result          = TestOut;
    return TestOut;
}

/**
 * @brief  Calculation of proportional integral controller
 * @note   
 * @param  Controller PI Controller pointer
 * @param  input Input value to be update control output
 * @retval 
 */
float PI_Calculate(PI_Controller* Controller,float input)
{
    float Error     = Controller->Reference - input;

    float TestOut   = Controller->Result + Error * (Controller->Kp + Controller->Ki*Controller->Ts) - Controller->PreviousError * Controller->Kp;

    TestOut         = (TestOut > Controller->MaxLim)?Controller->MaxLim:TestOut;
    TestOut         = (TestOut < Controller->MinLim)?Controller->MinLim:TestOut;

    Controller->PreviousError  = Error;
    Controller->Result          = TestOut;
    return TestOut;
}

/**
 * @brief  Calculation of proportional integral and derivative controller
 * @note   
 * @param  Controller PID Controller pointer
 * @param  input Input value to be update control output
 * @retval 
 */
float PID_Calculate(PID_Controller* Controller,float input)
{
    float Error     = Controller->Reference - input;

    float TestOut   = Controller->Result + Error * (Controller->Kp + Controller->Ki*Controller->Ts + Controller->Kd*Controller->Td) - Controller->PreviousError1 * (Controller->Kp + 2*Controller->Ki*Controller->Ts) + Controller->PreviousError2 * Controller->Kd * Controller->Td;

    TestOut         = (TestOut > Controller->MaxLim)?Controller->MaxLim:TestOut;
    TestOut         = (TestOut < Controller->MinLim)?Controller->MinLim:TestOut;

    Controller->PreviousError2  = Controller->PreviousError1;
    Controller->PreviousError1  = Error;
    Controller->Result     = TestOut;
    return TestOut;
}
/*
* Resonant Controllers
*/

/**
 * @brief  Initialization of the PR controller structure
 * @note   
 * @param  Controller: Pointer to the controller structure to be initialized
 * @param  Kp: Proportional gain to give to the PR controller
 * @param  Kr: Resonant gain to give to the PR controller
 * @param  w0: Base frequency 
 * @param  wC: Cut-off frequency
 * @param  MaxLim: Maximum saturation limit for the controller
 * @param  MinLim: Minimum saturation limit for the controller
 * @param  Ts: Sample time
 * @retval None
 */
void PR_init(PR_Controller* Controller, float Kp, float Kr, float w0, float wC,float MaxLim, float MinLim, float Ts)
{
    Controller->Kp = Kp;
    Controller->Kr = Kr;
    Controller->w0 = w0;
    Controller->wC = wC;
    Controller->MaxLim = MaxLim;
    Controller->MinLim = MinLim;
    Controller->Ts = Ts;

    Controller->A = (4*Kr+4*Kp)*Ts*wC+Kp*pow(Ts,2)*pow(w0,2)+4*Kp;
    Controller->B = 2*Kp*pow(Ts,2)*pow(w0,2)-8*Kp;
    Controller->C = (-4*Kr-4*Kp)*Ts*wC+Kp*pow(Ts,2)*pow(w0,2)+4*Kp;
    Controller->D = 4*Ts*wC+pow(Ts,2)*pow(w0,2)+4;
    Controller->E = 2*pow(Ts,2)*pow(w0,2)-8;
    Controller->F = -4*Ts*wC+pow(Ts,2)*pow(w0,2)+4;
    Controller->D1 = 1/Controller->D;
}

/**
 * @brief Reset of the PR Controller structure  
 * @note   
 * @param  Controller: Pointer to the controller structure
 * @retval Success value returned
 */
int PR_Reset(PR_Controller* Controller)
{
    Controller->Out_N1 = 0;
    Controller->Out_N2 = 0;
    Controller->Result = 0;
    Controller->Error_N1 = 0;
    Controller->Error_N2 = 0;
    return 0;
}

/**
 * @brief  Calculate the next output of the controller given the previous references and input
 * @note   Not Error just input data
 * @param  Controller: Pointer to controller structure
 * @param  input: Input data (! Not Error just input data!)
 * @retval Controller output
 */
float PR_Calculate(PR_Controller* Controller, float input)
{
    float Output;

    Controller->Result = Controller->D1*(Controller->A*(Controller->Reference - input) + Controller->B*Controller->Error_N1 + Controller->C*Controller->Error_N2) +
                                        (-Controller->E*Controller->Out_N1 -Controller->F*Controller->Out_N2);

    Controller->Result = (Controller->Result > Controller->MaxLim)?Controller->MaxLim:Controller->Result;
    Controller->Result = (Controller->Result < Controller->MinLim)?Controller->MinLim:Controller->Result;

    Controller->Error_N2 = Controller->Error_N1;
    Controller->Error_N1 = Controller->Reference - input;

    Controller->Out_N2 = Controller->Out_N1;
    Controller->Out_N1 = Controller->Result;

    return Controller->Result;
}