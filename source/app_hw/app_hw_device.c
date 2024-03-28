/*******************************************************************************
 * File Name: app_hw_device.c
 *
 * Description: This file contains functions for initialization and usage of
 *              device peripheral GPIO for LED and button. It also
 *              contains FreeRTOS timer implementations.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include <FreeRTOS.h>
#include <task.h>
#include "cybsp_types.h"
#include "timers.h"
#include "cyhal_gpio.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_hw_device.h"
#include "cycfg_pins.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define APP_BTN_PRESS_SHORT_MIN        (20)  /* Button push time min ms */
#define APP_BTN_PRESS_SHORT_MAX        (250) /* Button push time max ms */
#define APP_TIMEOUT_MS_BTN             (1)   /* Button timer for count push time */
#define APP_TIMEOUT_LED_INDICATE       (500) /* LED toggle timer for blinking */
#define MAXIMUM_LED_BLINK_COUNT        (11)  /* LED Blinking time period */
#define GPIO_INTERRUPT_PRIORITY        (4)   /* Interrupt priority for GPIO connected to button */
#define BTN_TASK_STACK_SIZE            (512u)/* Stack size for BTN task */
#define BTN_TASK_PRIORITY              (2)   /* Task Priority of BTN Task */

#ifndef CYBSP_USER_LED2
#define CYBSP_USER_LED2                 CYBSP_USER_LED
#endif
/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/* This timer is used to toggle LED to indicate button press type */
TimerHandle_t ms_timer_led_indicate;

/* ms_timer_btn is a periodic timer that ticks every millisecond.
 * This timer is used to measure the duration of button press events */
TimerHandle_t ms_timer_btn;

/* Variables to hold the timer count values */
uint8_t led_blink_count;

/* Handle of the button task */
TaskHandle_t button_handle;

/* Variable to keep track of LED blink count for long button press */
static uint8_t led_indicate_count;

/* For button press interrupt */
cyhal_gpio_callback_data_t btn_cb_data =
{
    .callback     = app_bt_gpio_interrupt_handler,
    .callback_arg = NULL
};

/**
 * Function Name: app_bt_timeout_ms_btn
 *
 * Function Description:
 *   @brief The function invoked on timeout of FreeRTOS millisecond timer.
 *          It is used to calculate button press duration.
 *
 *   @param Timerhandle_t timer_handle: unused
 *
 *   @return None
 *
 */
void app_bt_timeout_ms_btn(TimerHandle_t timer_handle)
{
    ccc_device_state.timer_count_ms++;
}

/**
 * Function Name: app_bt_timeout_led_indicate
 *
 * Function Description:
 *   @brief The function is invoked on timeout of FreeRTOS milliseconds timer.
 *          It is used to toggle the LED for 10 seconds for indication of advertisement is start.
 *
 *   @param Timerhandle_t timer_handle: unused
 *
 *   @return None
 *
 */
void app_bt_timeout_led_indicate(TimerHandle_t timer_handle)
{
    led_indicate_count++;
    cyhal_gpio_toggle(CYBSP_USER_LED2);

    if(led_indicate_count == MAXIMUM_LED_BLINK_COUNT)
    {
        xTimerStop(ms_timer_led_indicate, 0);
        cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_OFF);
        led_indicate_count = 0;
    }
}

/**
 * Function Name: app_bt_interrupt_config
 *
 * Function Description:
 *   @brief This function initializes a pin as input that triggers interrupt on
 *   falling edges.
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_interrupt_config(void)
{
    cy_rslt_t result=0;

    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN,
                             CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_PULLDOWN,
                             CYBSP_BTN_OFF);
    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &btn_cb_data);

    cyhal_gpio_enable_event(CYBSP_USER_BTN,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            true);
    UNUSED_VARIABLE(result);
}

/**
 * Function Name: app_bt_gpio_interrupt_handler
 *
 * Function Description:
 *   @brief GPIO interrupt handler.
 *
 *   @param void *handler_arg (unused)
 *   @param cyhal_gpio_irq_event_t (unused)
 *
 *   @return None
 *
 */
void app_bt_gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     if(CYHAL_GPIO_IRQ_FALL == event)
     {
         xTimerStartFromISR(ms_timer_btn, &xHigherPriorityTaskWoken);
     }
     if((CYHAL_GPIO_IRQ_RISE == event) || (CYHAL_GPIO_IRQ_FALL == event))
     {
         vTaskNotifyGiveFromISR(button_handle, &xHigherPriorityTaskWoken);
         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
     }
}

/**
 * Function Name: app_bt_hw_init
 *
 * Function Description:
 *   @brief This function initializes the LEDs and FreeRTOS Timers.
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_hw_init()
{
    cyhal_gpio_init(CYBSP_USER_LED1 , CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(CYBSP_USER_LED2 , CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Starting a log print timer for button press duration
     * and application traces  */
    ms_timer_btn = xTimerCreate("ms_timer",
                                pdMS_TO_TICKS(APP_TIMEOUT_MS_BTN),
                                pdTRUE,
                                NULL,
                                app_bt_timeout_ms_btn);
    xTimerStart(ms_timer_btn, 0);

    /* Starting a 1ms timer for indication LED used to show button press duration */
    ms_timer_led_indicate = xTimerCreate("ms_timer_led_indicate",
                                         pdMS_TO_TICKS(APP_TIMEOUT_LED_INDICATE),
                                         pdTRUE,
                                         NULL,
                                         app_bt_timeout_led_indicate);
    xTaskCreate(button_task,
                "Button task",
                BTN_TASK_STACK_SIZE,
                NULL,
                BTN_TASK_PRIORITY,
                &button_handle);
}

/**
 * Function Name: button_task
 *
 * Function Description:
 *   @brief This is a FreeRTOS task that handles the button press events.
 *
 *   @param None
 *
 *   @return None
 *
 */
void button_task(void *arg)
{
    uint32_t       btn_press_duration = 0;
    uint8_t        button_push        = 0;
    wiced_result_t result             = WICED_BT_SUCCESS;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(CYBSP_BTN_PRESSED == cyhal_gpio_read(CYBSP_USER_BTN))
        {
            ccc_device_state.btn_press_start = ccc_device_state.timer_count_ms;
            cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_ON);
        }
        else
        {
            cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_OFF);
            btn_press_duration = ccc_device_state.timer_count_ms - ccc_device_state.btn_press_start;
            /* Check if button press is available */
            if((btn_press_duration > APP_BTN_PRESS_SHORT_MIN) &&
               (btn_press_duration <= APP_BTN_PRESS_SHORT_MAX))
            {
                printf("button press is detected\n");
                button_push = TRUE;
            }

            if ((!ccc_device_state.start_scan) && (button_push))
            {
                result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, true, app_scan_result_handler);
                if (result != WICED_BT_PENDING)
                {
                    printf("Start Scan failed:%d\n", result);
                }
                else
                {
                    ccc_device_state.start_scan = 1;
                    printf("Start Scan:%d\n", result);
                    led_indicate_count = 0;
                    xTimerStart(ms_timer_led_indicate, 0);
                }
            }
            else if ((ccc_device_state.start_scan) && (button_push))
            {
                result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, true, app_scan_result_handler);
                if (result != WICED_BT_SUCCESS)
                {
                    printf("Stop Scan failed:%d\n", result);
                }
                else
                {
                    ccc_device_state.start_scan = 0;
                    printf("Stop Scan:%d\n", result);
                    led_indicate_count = 0;
                    xTimerStop(ms_timer_led_indicate, 0);
                }
            }
            /* Stop button jitter timer, start it again when button interrupt is detected */
            ccc_device_state.timer_count_ms  = 0;
            ccc_device_state.btn_press_start = 0;
            xTimerStop(ms_timer_btn, 0);
        }
    }
}
