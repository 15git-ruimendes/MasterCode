/**
 * @file Finite_State_Machines_XMC.h
 * @author Rui Mendes
 * @brief 
 * @version 0.1
 * @date 2022-09-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief  General Definitions of FSM.
 * @note   Requires changing and adaptation for different builds
 */

#define     NUM_FSM     11
#define     MASTER      0
#define     PRE_CHRG    1
#define     CONTROL     2
#define     INPUT       3
#define     OUTPUT      4
#define     ADC_MS      5

#define     OUT_MIN      750
#define     OUT_MAX      2500
#define     IN_VOLT_MIN  350
#define     IN_VOLT_MAX  2200
#define     PRE_CHRG_MIN 500
#define     PRE_CHRG_MAX 650
#define     T_PRE_CHR    5000
#define     T_RESET_FAIL 5000000

/**
 * @brief  Input values
 * @note   
 * @retval None
 */
float IN_VOLT  =  0;
float OUT_VOLT =  0;

/**
 * @brief  Output values
 * @note   
 * @retval None
 */
float Kpc       = 0;
float Kmain     = 0;
float KShunt1   = 0;
float KShunt2   = 0;

/**
 * @brief  Enable and start variables
 * @note   
 * @retval None
 */
int Enable      = 0;
int Start       = 0;
int DisCharge   = 0;

int ANY_FAIL   = 0;


/**
 * @brief  FSM structure defining state and counting time on state.
 * @note   
 *
 */
typedef struct 
{
    int State;
    int PrevState;
    int _TimeOn;    
}FSMachines;


// Main FSM Vector
FSMachines FSM[NUM_FSM];

// Tyipified function
void CalculateTransitions(void);