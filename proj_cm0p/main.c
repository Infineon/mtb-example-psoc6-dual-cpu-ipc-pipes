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
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


#include "cy_pdl.h"
#include "ipc_communication.h"
#include "cyhal.h"
#include "cybsp.h"
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
static const cy_stc_sysint_t mcwdt_int_cfg = {
    .intrSrc = (IRQn_Type) NvicMux7_IRQn,
    .cm0pSrc = srss_interrupt_mcwdt_0_IRQn,
    .intrPriority = 2
};

/* Watchdog timer configuration */
static const cy_stc_mcwdt_config_t mcwdt_cfg =
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

static volatile uint8_t msg_cmd = 0;

static ipc_msg_t ipc_msg = {              /* IPC structure to be sent to CM4  */
    .client_id  = IPC_CM0_TO_CM4_CLIENT_ID,
    .cpu_status = 0,
    .intr_mask   = USER_IPC_PIPE_INTR_MASK,
    .cmd        = IPC_CMD_STATUS,
    .value      = 0
};

/****************************************************************************
* Functions Prototypes
*****************************************************************************/
static void cm0p_msg_callback(uint32_t *msg);
static void mcwdt_handler(void);

int main(void)
{
    uint32_t random_number;
    cy_rslt_t result;
    cy_en_ipc_pipe_status_t ipc_status;
    cy_en_syspm_status_t syspm_status;
    cy_en_sysint_status_t sysint_status;
    cy_en_mcwdt_status_t mcwdt_status;
    cy_en_crypto_status_t crypt_status;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Init the IPC communication for CM0+ */
    setup_ipc_communication_cm0();

    /* Enable global interrupts */
    __enable_irq();

    /* Register callback to handle messages from CM4 */
    ipc_status = Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR,
                                            cm0p_msg_callback,
                                            IPC_CM4_TO_CM0_CLIENT_ID);
    if (ipc_status != CY_IPC_PIPE_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable CM4. CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

    for (;;)
    {
        syspm_status = Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        if (syspm_status != CY_SYSPM_SUCCESS)
        {
            CY_ASSERT(0);
        }
        /* Process IPC commands */
        switch (msg_cmd)
        {
            case IPC_CMD_INIT:
                /* Update clock settings */
                SystemCoreClockUpdate();

                /* Initialize the MCWDT Interrupt */
                sysint_status = Cy_SysInt_Init(&mcwdt_int_cfg, mcwdt_handler);
                if (sysint_status != CY_SYSINT_SUCCESS)
                {
                    CY_ASSERT(0);
                }
                NVIC_ClearPendingIRQ((IRQn_Type) mcwdt_int_cfg.intrSrc);
                NVIC_EnableIRQ((IRQn_Type) mcwdt_int_cfg.intrSrc);

                /* Init the MCWDT */
                mcwdt_status = Cy_MCWDT_Init(MCWDT_HW, &mcwdt_cfg);
                if (mcwdt_status != CY_MCWDT_SUCCESS)
                {
                    CY_ASSERT(0);
                }
                Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR0);     
                break;

            case IPC_CMD_START:
                Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR0, 0u);
                crypt_status = Cy_Crypto_Core_Enable(CRYPTO);
                if (crypt_status != CY_CRYPTO_SUCCESS)
                {
                    CY_ASSERT(0);
                }
                break;

            case IPC_CMD_STOP:
                Cy_MCWDT_Disable(MCWDT_HW, CY_MCWDT_CTR0, 0u);
                crypt_status = Cy_Crypto_Core_Disable(CRYPTO);
                if (crypt_status != CY_CRYPTO_SUCCESS)
                {
                    CY_ASSERT(0);
                }
                break;

            case IPC_CMD_STATUS:

                /* Generate a random number */
                crypt_status = Cy_Crypto_Core_Trng(CRYPTO, GARO31_INITSTATE,
                                            FIRO31_INITSTATE, 
                                            MAX_TRNG_BIT_SIZE, &random_number);
                if (crypt_status != CY_CRYPTO_SUCCESS)
                {
                    CY_ASSERT(0);
                }

                ipc_msg.value = random_number;

                /* Send the random number to CM4 to be printed */
                ipc_status = Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM4,
                                        USER_IPC_PIPE_EP_ADDR_CM0,
                                        (uint32_t *) &ipc_msg, NULL);
                if (ipc_status != CY_IPC_PIPE_SUCCESS)
                {
                    CY_ASSERT(0);
                }
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
static void cm0p_msg_callback(uint32_t *msg)
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
static void mcwdt_handler(void)
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

/*******************************************************************************
* Function Name: cybsp_register_custom_sysclk_pm_callback
********************************************************************************
* Summary:
*   Custom power management callback for CM0+ core which is registered by cybsp_init().
*   This avoids conflict with default power management callback registered on 
*   CM4 core.
*
*******************************************************************************/
cy_rslt_t cybsp_register_custom_sysclk_pm_callback(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    return result;
}

/* [] END OF FILE */
