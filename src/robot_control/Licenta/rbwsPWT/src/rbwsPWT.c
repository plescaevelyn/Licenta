/*
 * rbwsPWT.c
 *
 *  Created on: Apr 16, 2024
 *      Author: PLE1CLJ
 */

/************/
/* Includes */
/************/
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_pwt.h"

#include "fsl_common.h"

#include "rbwsPWT.h"

/***************
 * Definitions
 ***************/
/* Interrupt number for the PWT instance used */
#define PWT_INTERRUPT_NUMBER PWT_LPTMR0_IRQn
#define PWT_EXAMPLE_HANDLER  PWT_LPTMR0_IRQHandler

/* Get source clock for PWT driver */
#define PWT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/************************/
/* Variable definitions */
/************************/
volatile bool busyWait;
volatile bool overflowFlag;
pwt_config_t pwtConfig;
uint16_t pulseWidth = 0;
uint8_t reg;

/****************************/
/* Function implementations */
/****************************/
/*!
 * @brief ISR for PWT interrupt
 *
 * This function changes the state of busyWait.
 */
void PWT_EXAMPLE_HANDLER(void)
{
    if (PWT_GetStatusFlags(PWT) & kPWT_PulseWidthValidFlag)
    {
        /*
         * Disable PWT pulse ready interrupt, ;
         * we do not want to clear the PWTRDY status bit before reading the data
         */
        PWT_DisableInterrupts(PWT, kPWT_PulseWidthReadyInterruptEnable);
        busyWait = false;
    }

    if (PWT_GetStatusFlags(PWT) & kPWT_CounterOverflowFlag)
    {
        /* Clear overflow flag */
        PWT_ClearStatusFlags(PWT, kPWT_CounterOverflowFlag);
        overflowFlag = true;
    }
    SDK_ISR_EXIT_BARRIER;
}

/**
 * @brief Initialises the PWT module.
 * @info kPWT_InputPort_1 used for Servo PWT, kPWT_InputPort_2 used for ESC PWT
 *
 * @param The PWT port that is to be initialized
 */
void rbwsPWT_init(pwt_input_select_t pwt_port)
{
    pwt_config_t pwtConfig;

    /* Deinit PWT to reconfigure */
    PWT_Deinit(PWT);

    /* Init PWT */
    pwtConfig.inputSelect = pwt_port;
    pwtConfig.clockSource = PWT_SOURCE_CLOCK;
    pwtConfig.enableFirstCounterLoad = true;
    pwtConfig.prescale = kPWT_Prescale_Divide_16;

    PWT_Init(PWT, &pwtConfig);

    /* Start the PWT counter */
    PWT_StartTimer(PWT);
}

/**
 * @brief This function reads the pulse from the FS-i6B receiver, returns the pulse width
 * and displays whether the buggy is driving in forward or backward mode or whether it is
 * in a neutral state (the buggy doesn't move).
 *
 * @param mode 0 controls the ESC and mode 1 controls the Servo motor
 */
void rbwsPWT_readPulse(uint8_t mode)
{
    busyWait     = true;
    overflowFlag = false;

    /* Enable PWT pulse ready interrupt */
    PWT_EnableInterrupts(PWT, kPWT_PulseWidthReadyInterruptEnable);

    /* Wait till ready interrupt occurs */
    while (busyWait)
    {
    }

    if (overflowFlag)
    {
        reg = ((PWT->CR) & (PWT_CR_LVL_MASK | PWT_CR_TGL_MASK)) >> PWT_CR_LVL_SHIFT;

        switch (reg)
        {
            case 0:
                PRINTF("\r\nLow overflow (0 duty ratio), signal stayed low\r\n");
                break;
            case 1:
                PRINTF("\r\nHigh overflow (100% duty ratio), signal stayed high\r\n");
                break;
            case 2:
                PRINTF("\r\nToggled Low overflow\r\n");
                break;
            default:
                PRINTF("\r\nToggled High overflow\r\n");
                break;
        }
    }
    else
    {
        pulseWidth = PWT_ReadPositivePulseWidth(PWT);
        pulseWidth = (uint16_t) (16.1290322581 * COUNT_TO_USEC(pulseWidth, PWT_SOURCE_CLOCK));
        PRINTF("\r\nPositive pulse width = %d usec\r\n", pulseWidth);

        switch(mode)
        {
        	case 0:
        		uint16_t var1;
        		if (pulseWidth >= 1450 && pulseWidth <= 1550)
        		{
        			/* Neutral state */
        			PRINTF("Buggy is in neutral state.\n");
        			var1 = 75;
        			rbwsMotorControl_Set_Motor_DutyCycle(var1);
        		}
        		else if (pulseWidth > 1550)
        		{
        			/* Forward driving mode */
        			PRINTF("Buggy is in forward driving mode.\n");

        			var1 = (uint16_t)(0.025*pulseWidth + 37.5);
        			rbwsMotorControl_Set_Motor_DutyCycle(var1);
        		}
        		else if (pulseWidth < 1450)
        		{
        			/* Backwards driving mode. In this case, a backwards-neutral-backwards mode is required */
        			PRINTF("Buggy is in backwards driving mode.\n");

        			var1 = (uint16_t)(0.025*pulseWidth + 37.5);
        			rbwsMotorControl_Set_Motor_DutyCycle(var1);
        		}
        		else
        		{
        			 PRINTF("Undefined!\n");
        		}
        		break;
        	case 1:
        		if (pulseWidth < 1000 || pulseWidth > 2000)
        		{
        			if(pulseWidth < 110)
        			{
        				pulseWidth = 1900;
        			}
         			PRINTF("No signal received from the RC!\n");
        		}
        		uint16_t var = (uint16_t)(-0.025*pulseWidth + 112.5 - 8);
        		rbwsMotorControl_Set_Servo_DutyCycle(var);
        	break;
        	default:
        		PRINTF("Undefined mode!\n");
        	break;
        }
    }
    /* Clear pulse ready flag */
    PWT_ClearStatusFlags(PWT, kPWT_PulseWidthValidFlag);
}

/*!
 * @brief Configures the PWT and reads the pulses from the ESC and the servo motor alternatively.
 */
void rbwsPWT_main()
{
    PWT_GetDefaultConfig(&pwtConfig);
    PWT_Init(PWT, &pwtConfig);

    rbwsPWT_init(kPWT_InputPort_1);

    /* Enable at the NVIC */
    EnableIRQ(PWT_INTERRUPT_NUMBER);

    /* This loop will set the print the pulse width */
    /* ESC Handling */
    rbwsPWT_init(kPWT_InputPort_1);
    rbwsPWT_readPulse(0);

    /* Servo Motor Handling */
    rbwsPWT_init(kPWT_InputPort_2);
	rbwsPWT_readPulse(1);
}
