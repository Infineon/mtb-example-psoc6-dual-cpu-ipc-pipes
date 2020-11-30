/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM4 in the the Dual CPU IPC Pipes 
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
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "ipc_communication.h"

#include <stdio.h>

/****************************************************************************
* Constants
*****************************************************************************/
#define BTN_MSG_START   1u
#define BTN_MSG_STOP    2u

#define SEND_IPC_MSG(x) ipc_msg.cmd = x; \
                        Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM0, \
                                                USER_IPC_PIPE_EP_ADDR_CM4, \
                                                (void *) &ipc_msg, 0);     

/****************************************************************************
* Functions Prototypes
*****************************************************************************/
void cm4_msg_callback(uint32_t *msg);
void button_isr_handler(void *arg, cyhal_gpio_event_t event);

/****************************************************************************
* Global Variables
*****************************************************************************/
/* IPC structure to be sent to CM0+ */
ipc_msg_t ipc_msg = {               
    .client_id  = IPC_CM4_TO_CM0_CLIENT_ID,
    .cpu_status = 0,
    .intr_mask  = USER_IPC_PIPE_INTR_MASK,
    .cmd        = IPC_CMD_INIT,
};

/* Message variables */
volatile bool msg_flag = false;
volatile uint32_t msg_value;
volatile uint32_t button_flag;

/* MCWDT Object [used by CM0+] */
cyhal_resource_inst_t mcwdt_0 =
{
    .type = CYHAL_RSC_LPTIMER,
    .block_num = 0
};

int main(void)
{
    cy_rslt_t result;

    /* Init the IPC communication for CM4 */
    setup_ipc_communication_cm4();

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Reserve the MCWDT used by the CM0+ to avoid conflicts if a LPTIMER is 
       to be added in the CM4 */
    cyhal_hwmgr_reserve(&mcwdt_0);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User Button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_BOTH, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, button_isr_handler, NULL);

    /* Register the Message Callback */
    Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR,
                                 cm4_msg_callback,
                                 IPC_CM0_TO_CM4_CLIENT_ID);   

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** \
    IPC Pipes Example \
    ****************** \r\n\n");

    printf("<Press the User Button to Create Random Numbers>\r\n");

    SEND_IPC_MSG(IPC_CMD_INIT);    

    for (;;)
    {
        cyhal_system_sleep();

        /* Check if a message was received from CM0+ */
        if (msg_flag)
        {
            msg_flag = false;

            /* Print random number received from CM0+ */
            printf(" 0x%.8x\n\r", (unsigned int) msg_value);
        }

        /* Check if the button was pressed */
        if (button_flag == BTN_MSG_STOP)
        {
            printf("<---Stop--->\n\r");
            SEND_IPC_MSG(IPC_CMD_STOP);
            
            /* Clear flag */
            button_flag = 0;
        }
        else if (button_flag == BTN_MSG_START)
        {
            printf("\n\r<---Start-->\n\r");
            SEND_IPC_MSG(IPC_CMD_START);
            
            /* Clear flag */
            button_flag = 0;
        }
    }
}

/*******************************************************************************
* Function Name: cm4_msg_callback
********************************************************************************
* Summary:
*   Callback function to execute when receiving a message from CM0+ to CM4.
*
* Parameters:
*   msg: message received
*
*******************************************************************************/
void cm4_msg_callback(uint32_t *msg)
{
    ipc_msg_t *ipc_recv_msg;

    if (msg != NULL)
    {
        /* Cast received message to the IPC message structure */
        ipc_recv_msg = (ipc_msg_t *) msg;

        /* Extract the message value */
        msg_value = ipc_recv_msg->value;

        /* Set message flag */
        msg_flag = true;
    }
    
}

/*******************************************************************************
* Function Name: button_isr_handler
********************************************************************************
* Summary:
*  Button ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void button_isr_handler(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;

    if (event & CYHAL_GPIO_IRQ_RISE)
    {
        button_flag = BTN_MSG_STOP;
    } 
    else if (event & CYHAL_GPIO_IRQ_FALL)
    {
        button_flag = BTN_MSG_START;
    }
}

/* [] END OF FILE */
