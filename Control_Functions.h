/**
* @brief    PSim Enabled control library
* @file     Control_Schemas_XMC.h
* @author   Rui Mendes
* @date     26-09-2022
*/

#ifndef CONTROL_H_
#define CONTROL_H_

/**
 * @brief  Structure to create a P controller 
 * @note   
 * @retval None
 */
typedef struct
{
    float Reference;
    float Kp;

    float MaxLim;
    float MinLim;

    float Result;
}Prop_Controller;

/**
 * @brief Structure to create a PI controller 
 * @note   
 * @retval None
 */
typedef struct
{
    float Reference;
    float Kp;
    float Ki;

    float Ts;
    float PreviousError;
    int IntegralSat;

    float MaxLim;
    float MinLim;

    float Result;
}PI_Controller;

/**
 * @brief  Structure to create a PID controller 
 * @note   
 * @retval None
 */
typedef struct
{
    float Reference;
    float Kp;
    float Ki;
    float Kd;

    float Ts;
    float Td;
    float PreviousError1;
    float PreviousError2;
    int IntegralSat;

    float MaxLim;
    float MinLim;

    float Result;
}PID_Controller;

/**
 * @brief Structure for a digital proportional resonant controller
 * @note   
 * @retval None
 */
typedef struct 
{
    /*
    *  Public Variables
    */
    float Reference;

    float Kp;
    float Kr;

    float Ts;
    float w0;
    float wC;

    float MaxLimit;

    float Error_N1;
    float Error_N2;

    float Out_N1;
    float Out_N2;

    float MaxLim;
    float MinLim;
    float Saturation;

    float Result;
   
    /*
    * Private Variables
    */
   float A;
   float B;
   float C;
   float D;
   float E;
   float F;
   float G;
   float D1;
}PR_Controller;


// Without Structure

float P_Controller(float In,float Ref,float Kp,float MaxLim,float MinLim,float* Output);

float Pi_Controller(float In,float Ref, float Kp,float Ki,float Ts,float* PrevError,float MaxLim,float MinLim,float* Output);

float Pid_Controller(float In,float Ref,float Kp,float Ki,float Kd,float Ts,float* PrevError1,float* PrevError2,float MaxLim,float MinLim,float* Output);

// With Structure

void P_init(Prop_Controller* Controller, float Kp, float Reference, float MaxLim,float MinLim);

void PI_init(PI_Controller* Controller, float Kp, float Ki, float Reference, float MaxLim,float MinLim,float Ts);

void PID_init(PID_Controller* Controller, float Kp, float Ki, float Kd, float Reference, float MaxLim,float MinLim,float Ts);

void PR_init(PR_Controller* Controller, float Kp, float Kr, float w0, float wC,float MaxLim, float MinLim, float Ts);

// Reset Functions

int PI_Reset(PI_Controller* Controller);

int PID_Reset(PID_Controller* Controller);

int PR_Reset(PR_Controller* Controller);

// Calculate

float P_Calculate(Prop_Controller* Controller,float input);

float PI_Calculate(PI_Controller* Controller,float input);

float PID_Calculate(PID_Controller* Controller,float input);

float PR_Calculate(PR_Controller* Controller, float input);

#endif