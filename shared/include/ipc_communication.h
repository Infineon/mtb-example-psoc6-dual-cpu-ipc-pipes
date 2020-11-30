/*****************************************************************************
* File Name  : ipc_communication.h
*
* Description: This file contains definitions of constants and structures for
*              setting up user pipe and function prototypes for configuring
*              system and user IPC pipe.
*
*******************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation.
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
#ifndef SOURCE_IPC_COMMUNICATION_H
#define SOURCE_IPC_COMMUNICATION_H

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Endpoint indexes in the pipe array */
#define USER_IPC_PIPE_EP_ADDR_CM0       (2UL)
#define USER_IPC_PIPE_EP_ADDR_CM4       (3UL)

#if (CY_CPU_CORTEX_M0P)
    #define USER_IPC_PIPE_EP_ADDR       USER_IPC_PIPE_EP_ADDR_CM0
#else
    #define USER_IPC_PIPE_EP_ADDR       USER_IPC_PIPE_EP_ADDR_CM4
#endif  /* (CY_CPU_CORTEX_M0P) */

/* Number of clients on each endpoint */
#define USER_IPC_PIPE_CLIENT_CNT        (4UL)

/* IPC data channel for User PIPE EP0 */
#define USER_IPC_PIPE_CHAN_EP0          (8UL)

/* IPC data channel for User PIPE EP1 */
#define USER_IPC_PIPE_CHAN_EP1          (9UL)

/* Notifier EP0 */
#define USER_IPC_PIPE_INTR_EP0          (8UL)

/* Notifier EP1 */
#define USER_IPC_PIPE_INTR_EP1          (9UL)

/* CM0+ NVIC MUX for IPC */
#define USER_IPC_PIPE_INTR_MUX_EP0      (2UL)

#define USER_IPC_PIPE_INTR_MASK         (uint32_t)((1UL << USER_IPC_PIPE_CHAN_EP0) |\
                                                 (1UL << USER_IPC_PIPE_CHAN_EP1))
#define USER_IPC_PIPE_EP0_CONFIG        (_VAL2FLD(CY_IPC_PIPE_CFG_IMASK, USER_IPC_PIPE_INTR_MASK) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_INTR,  USER_IPC_PIPE_INTR_EP0) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_CHAN,  USER_IPC_PIPE_CHAN_EP0))
#define USER_IPC_PIPE_EP1_CONFIG        (_VAL2FLD(CY_IPC_PIPE_CFG_IMASK, USER_IPC_PIPE_INTR_MASK) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_INTR,  USER_IPC_PIPE_INTR_EP1) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_CHAN,  USER_IPC_PIPE_CHAN_EP1))

#define IPC_CM0_TO_CM4_CLIENT_ID        (1U)
#define IPC_CM4_TO_CM0_CLIENT_ID        (2U)

#define IPC_CMD_INIT                    0x81
#define IPC_CMD_START                   0x82
#define IPC_CMD_STOP                    0x83
#define IPC_CMD_STATUS                  0x41

/*******************************************************************************
* Enumerations
*******************************************************************************/
typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     client_id;
    uint8_t     cpu_status;
    uint16_t    intr_mask;
    uint8_t     cmd;
    uint32_t    value;
} ipc_msg_t ;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
void setup_ipc_communication_cm0(void);
void user_ipc_pipe_isr_cm0(void);
void setup_ipc_communication_cm4(void);
void user_ipc_pipe_isr_cm4(void);


#endif /* SOURCE_IPC_COMMUNICATION_H */

/* [] END OF FILE */
