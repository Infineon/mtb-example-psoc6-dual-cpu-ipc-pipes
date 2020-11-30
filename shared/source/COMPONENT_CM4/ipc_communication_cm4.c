/*****************************************************************************
* File Name  : ipc_communication_cm4.c
*
* Description: This file contains function definitions for setting up system
*              IPC communication and user IPC pipe.
*
*
*******************************************************************************
* Copyright (2020, Cypress Semiconductor Corporation.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the Right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "ipc_communication.h"
#include "system_psoc6.h"


/*******************************************************************************
 * Function definitions
 ******************************************************************************/


/*******************************************************************************
* Function Name: setup_ipc_communication_cm4
********************************************************************************
* Summary:
*        - Initializes IPC Semaphore.
*        - Configures CY_PIPE used for System calls, Flash, and BLE.
*        - Configures USER_PIPE used in the example for sending CapSense touch
*          detection data.
*        - Initializes both CY_PIPE and USER_PIPE.
*        - Initializes flash after CY_PIPE is initialized.
*
*******************************************************************************/
void setup_ipc_communication_cm4(void)
{
    #ifdef __CM0P_PRESENT
    #if (__CM0P_PRESENT == 0)
        /* Allocate and initialize semaphores for the system operations. */
        static uint32_t ipcSemaArray[CY_IPC_SEMA_COUNT / CY_IPC_SEMA_PER_WORD];
        (void) Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, CY_IPC_SEMA_COUNT, ipcSemaArray);
    #else
        (void) Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, 0ul, NULL);
    #endif /* (__CM0P_PRESENT) */
#else
    (void) Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, 0ul, NULL);
#endif /* __CM0P_PRESENT */

    /* Create an array of endpoint structures */
    static cy_stc_ipc_pipe_ep_t systemIpcPipeEpArray[CY_IPC_MAX_ENDPOINTS];

    Cy_IPC_Pipe_Config(systemIpcPipeEpArray);

    static cy_ipc_pipe_callback_ptr_t systemIpcPipeSysCbArray[CY_SYS_CYPIPE_CLIENT_CNT];

    static const cy_stc_ipc_pipe_config_t systemIpcPipeConfigCm4 =
    {
    /* .ep0ConfigData */
        {
            CY_IPC_INTR_CYPIPE_EP0,         /* .ipcNotifierNumber    */  
            CY_SYS_INTR_CYPIPE_PRIOR_EP0,   /* .ipcNotifierPriority  */  
            CY_SYS_INTR_CYPIPE_MUX_EP0,     /* .ipcNotifierMuxNumber */  
            CY_IPC_EP_CYPIPE_CM0_ADDR,      /* .epAddress            */  
            CY_SYS_CYPIPE_CONFIG_EP0        /* .epConfig             */  
        },
    /* .ep1ConfigData */
        {
            CY_IPC_INTR_CYPIPE_EP1,         /* .ipcNotifierNumber    */
            CY_SYS_INTR_CYPIPE_PRIOR_EP1,   /* .ipcNotifierPriority  */
            0u,                             /* .ipcNotifierMuxNumber */
            CY_IPC_EP_CYPIPE_CM4_ADDR,      /* .epAddress            */
            CY_SYS_CYPIPE_CONFIG_EP1        /* .epConfig             */
        },
        CY_SYS_CYPIPE_CLIENT_CNT,           /* .endpointClientsCount     */   
        systemIpcPipeSysCbArray,            /* .endpointsCallbacksArray  */   
        &Cy_SysIpcPipeIsrCm4                /* .userPipeIsrHandler       */  
    };

    static cy_ipc_pipe_callback_ptr_t user_ipc_pipe_cb_array[USER_IPC_PIPE_CLIENT_CNT];

    static const cy_stc_ipc_pipe_config_t user_ipc_pipe_config_cm4 =
    {
        /* .ep2ConfigData */
        {
            USER_IPC_PIPE_INTR_EP0,        /* .ipcNotifierNumber       */
            1UL,                           /* .ipcNotifierPriority     */
            USER_IPC_PIPE_INTR_MUX_EP0,    /* .ipcNotifierMuxNumber    */
            USER_IPC_PIPE_EP_ADDR_CM0,     /* .epAddress               */
            USER_IPC_PIPE_EP0_CONFIG       /* .epConfig                */
        },
        /* .ep3ConfigData */
        {
            USER_IPC_PIPE_INTR_EP1,        /* .ipcNotifierNumber       */
            1UL,                           /* .ipcNotifierPriority     */
            0UL,                           /* .ipcNotifierMuxNumber    */
            USER_IPC_PIPE_EP_ADDR_CM4,     /* .epAddress               */
            USER_IPC_PIPE_EP1_CONFIG       /* .epConfig                */
        },
        USER_IPC_PIPE_CLIENT_CNT,          /* .endpointClientsCount     */  
        user_ipc_pipe_cb_array,           /* .endpointsCallbacksArray  */  
        &user_ipc_pipe_isr_cm4               /* .userPipeIsrHandler       */  
    };

    
    Cy_IPC_Pipe_Init(&systemIpcPipeConfigCm4);

#if defined(CY_DEVICE_PSOC6ABLE2)
    Cy_Flash_Init();
#endif /* defined(CY_DEVICE_PSOC6ABLE2) */

    Cy_IPC_Pipe_Init(&user_ipc_pipe_config_cm4);
}


/*******************************************************************************
* Function Name: Cy_SysIpcPipeIsrCm4
****************************************************************************//**
*
* This is the interrupt service routine for the system pipe.
*
*******************************************************************************/
void Cy_SysIpcPipeIsrCm4(void)
{
    Cy_IPC_Pipe_ExecuteCallback(CY_IPC_EP_CYPIPE_CM4_ADDR);
}


/*******************************************************************************
* Function Name: user_ipc_pipe_isr_cm4
********************************************************************************
* Summary: User IRQ handler function that is called when IPC receives data from
*          CM0+ to CM4 through USER_PIPE.
*
*******************************************************************************/
void user_ipc_pipe_isr_cm4(void)
{
    Cy_IPC_Pipe_ExecuteCallback(USER_IPC_PIPE_EP_ADDR);
}
