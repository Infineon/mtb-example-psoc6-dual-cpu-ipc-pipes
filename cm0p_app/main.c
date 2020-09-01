/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM0+ in the the Dual CPU IPC Pipes 
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) (2020), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "ipc_def.h"

/****************************************************************************
* Constants
*****************************************************************************/
#define MCWDT_HW                MCWDT_STRUCT0
/* TRNG constants */
#define GARO31_INITSTATE        (0x04c11db7)
#define FIRO31_INITSTATE        (0x04c11db7)
#define MAX_TRNG_BIT_SIZE       (32UL)

/****************************************************************************
* Global variables
*****************************************************************************/
const cy_stc_sysint_t mcwdt_int_cfg = {
    .intrSrc = (IRQn_Type) NvicMux7_IRQn,
    .cm0pSrc = srss_interrupt_mcwdt_0_IRQn,
    .intrPriority = 2
};

/* Watchdog timer configuration */
const cy_stc_mcwdt_config_t mcwdt_cfg =
{
    .c0Match     = 32768,
    .c1Match     = 32768,
    .c0Mode      = CY_MCWDT_MODE_INT,
    .c1Mode      = CY_MCWDT_MODE_NONE ,
    .c2ToggleBit = 16,
    .c2Mode      = CY_MCWDT_MODE_NONE ,
    .c0ClearOnMatch = true,
    .c1ClearOnMatch = false,
    .c0c1Cascade = false,
    .c1c2Cascade = false
};

volatile uint8_t msg_cmd = 0;

ipc_msg_t ipc_msg = {              /* IPC structure to be sent to CM4  */
    .clientId = IPC_CM0_TO_CM4_CLIENT_ID,
    .userCode = 0,
    .intrMask = CY_SYS_CYPIPE_INTR_MASK,
    .cmd      = IPC_CMD_STATUS,
    .value    = 0
};

/****************************************************************************
* Functions Prototypes
*****************************************************************************/
void cm0p_msg_callback(uint32_t *msg);
void mcwdt_handler(void);

int main(void)
{
    uint32_t random_number;

    /* Enable global interrupts */
    __enable_irq();

    /* Register callback to handle messages from CM4 */
    Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR,
                                 cm0p_msg_callback,
                                 IPC_CM4_TO_CM0_CLIENT_ID);

    /* Enable CM4. CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);   

    for (;;)
    {
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        
        /* Process IPC commands */
        switch (msg_cmd)
        {
            case IPC_CMD_INIT:
                /* Update clock settings */
                SystemCoreClockUpdate();

                /* Initialize the MCWDT Interrupt */
                Cy_SysInt_Init(&mcwdt_int_cfg, mcwdt_handler);
                NVIC_ClearPendingIRQ((IRQn_Type) mcwdt_int_cfg.intrSrc);
                NVIC_EnableIRQ((IRQn_Type) mcwdt_int_cfg.intrSrc);

                /* Init the MCWDT */
                Cy_MCWDT_Init(MCWDT_HW, &mcwdt_cfg);
                Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR0);     
                break;

            case IPC_CMD_START:
                Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR0, 0u);
                Cy_Crypto_Core_Enable(CRYPTO);
                break;

            case IPC_CMD_STOP:
                Cy_MCWDT_Disable(MCWDT_HW, CY_MCWDT_CTR0, 0u);
                Cy_Crypto_Core_Disable(CRYPTO);
                break;

            case IPC_CMD_STATUS:

                /* Generate a random number */
                Cy_Crypto_Core_Trng(CRYPTO, GARO31_INITSTATE, 
                                            FIRO31_INITSTATE, 
                                            MAX_TRNG_BIT_SIZE, &random_number);

                ipc_msg.value = random_number;

                /* Send the random number to CM4 to be printed */
                Cy_IPC_Pipe_SendMessage(CY_IPC_EP_CYPIPE_CM4_ADDR,
                                        CY_IPC_EP_CYPIPE_CM0_ADDR,
                                        (uint32_t *) &ipc_msg, NULL);          
                break;

            default:
                break;
        }
        /* Clear command */
        msg_cmd = 0;
    }
}

/*******************************************************************************
* Function Name: cm0p_msg_callback
********************************************************************************
* Summary:
*   Callback function to execute when receiving a message from CM4 to CM0+.
*
* Parameters:
*   msg: message received
*
*******************************************************************************/
void cm0p_msg_callback(uint32_t *msg)
{
    ipc_msg_t *ipc_recv_msg;

    if (msg != NULL)
    {
        /* Cast the message received to the IPC structure */
        ipc_recv_msg = (ipc_msg_t *) msg;

        /* Extract the command to be processed in the main loop */
        msg_cmd = ipc_recv_msg->cmd;
    }
}

/*******************************************************************************
* Function Name: mcwdt_handler
********************************************************************************
* Summary:
*   Watchdog handler to periodically wake up the CM0+.
*
*******************************************************************************/
void mcwdt_handler(void)
{
    uint32 mcwdtIsrMask;

    /* Get the Watchdog Interrupt Status */
    mcwdtIsrMask = Cy_MCWDT_GetInterruptStatus(MCWDT_HW);

    if(0u != (CY_MCWDT_CTR0 & mcwdtIsrMask))
    {
        /* Clear Watchdog Interrupt */
        Cy_MCWDT_ClearInterrupt(MCWDT_HW, CY_MCWDT_CTR0);  

        /* Set the message command to be processed in the main loop */
        msg_cmd = IPC_CMD_STATUS;
    }  
}  

/* [] END OF FILE */
