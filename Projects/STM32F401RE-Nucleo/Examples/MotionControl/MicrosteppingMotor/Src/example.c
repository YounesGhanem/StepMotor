/**
  ******************************************************************************
  * @file       example.c
  * @date       01/10/2014 12:00:00
  * @brief      Example functions for the X-NUCLEO-IHM02A1
******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "example.h"
#include "example_usart.h"
#include "params.h"
#include "xnucleoihm02a1.h"

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @addtogroup Example
  * @{
  */

/**
  * @defgroup   ExamplePrivateFunctions
  * @brief      Example Private Functions.
  * @{
  */

uint16_t BSP_ST1S14_PGOOD(void);
uint32_t usrPow(uint8_t base, uint8_t exponent);

/**
  * @}
  */ /* End of ExamplePrivateFunctions */

/**
  * @addtogroup ExamplePrivateFunctions
  * @brief      Example Private Functions.
  * @{
  */

/**
  * @addtogroup ExampleExportedFunctions
  * @brief      Example Exported Functions.
  * @{
  */

/**
  * @brief  Example no.1 for X-NUCLEO-IHM02A1.
  * @note	Perform a complete motor axis revolution as MPR_1 equal movements,
  *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
  *			At the end of each movement there is a delay of DELAY_1 ms.
  *     	After each motor has performed a complete revolution there is a
  *			delay of DELAY_2 ms.
  *			Now all motors for each X-NUCLEO-IHM02A1 will start at the same
  *			time.
  *			They are going to run at INIT_SPEED for DELAY_3 ms.
  *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardStop
  *			at the same time.
  *			Perform a complete motor axis revolution as MPR_2 equal movements,
  *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
  *			At the end of each movement there is a delay of DELAY_1 ms.
  *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardHiZ
  *			at the same time.
  */
void MicrosteppingMotor_Example_01(void)
{
#define MPR_1     4			  //!< Motor Movements Per Revolution 1st option
#define MPR_2     8			  //!< Motor Movements Per Revolution 2nd option
#define DELAY_1   1000		//!< Delay time 1st option
#define DELAY_2   2500		//!< Delay time 2nd option
#define DELAY_3   10000   //!< Delay time 3rd option
  
  uint32_t Step;
  uint32_t Speed;
  uint8_t MovementPerRevolution;
  uint8_t i;
  uint8_t board, device;
  
  uint8_t id;
  
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
  #ifdef NUCLEO_USE_USART
  USART_Transmit(&huart2, (uint8_t *)"Initial values for registers:\r\n");
  USART_PrintAllRegisterValues();
  #endif

  /* Setup each X-NUCLEO-IHM02A1 Expansion Board ******************************/
  
  /* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }
  
  #ifdef NUCLEO_USE_USART
  USART_Transmit(&huart2, (uint8_t *)"Custom values for registers:\r\n");
  USART_PrintAllRegisterValues();
  #endif
  
  /****************************************************************************/
  
  MovementPerRevolution = MPR_1;
  for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
      MotorParameterDataSingle = MotorParameterDataGlobal+((board*L6470DAISYCHAINSIZE)+device);
      Step = ((uint32_t)MotorParameterDataSingle->fullstepsperrevolution * usrPow(2, MotorParameterDataSingle->step_sel)) / MovementPerRevolution;
      
      for (i=0; i<MovementPerRevolution; i++)
      {
        StepperMotorBoardHandle->Command->Move(board, device, L6470_DIR_FWD_ID, Step);
        while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
        HAL_Delay(DELAY_1);
      }
    }
  }
  
  HAL_Delay(DELAY_2);
  
  for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
      MotorParameterDataSingle = MotorParameterDataGlobal+((board*L6470DAISYCHAINSIZE)+device);
      
      /* Set Speed */
      Speed = Step_s_2_Speed(MotorParameterDataSingle->speed);
      
      /* Prepare the stepper driver to be ready to perform a command */
      StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareRun(device, L6470_DIR_FWD_ID, Speed);
    }
    
    StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
  }
  
  HAL_Delay(DELAY_3);

  for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Prepare the stepper driver to be ready to perform a command */
      StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
    }
    
    StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
  }

  MovementPerRevolution = MPR_2;
  for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
      MotorParameterDataSingle = MotorParameterDataGlobal+((board*L6470DAISYCHAINSIZE)+device);
      Step = ((uint32_t)MotorParameterDataSingle->fullstepsperrevolution * usrPow(2, MotorParameterDataSingle->step_sel)) / MovementPerRevolution;
      
      for (i=0; i<MovementPerRevolution; i++)
      {
        StepperMotorBoardHandle->Command->Move(board, device, L6470_DIR_FWD_ID, Step);
        while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
        HAL_Delay(DELAY_1);
      }
    }
  }
  
  for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Prepare the stepper driver to be ready to perform a command */
      StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
    }
    
    StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
  }
  
  /* Switch on the user LED */
    BSP_LED_On(LED2);
}





#define MOTOR_STEPS 200
#define DELAY_BETWEEN_MOVEMENTS 1000
/**
 * @fn void ControlMotorWithButton(void)
 * @brief Controls the stepper motor direction based on button press using polling.
 *
 * This function initializes the motor and then enters a loop where it continuously
 * checks the state of a button. On detecting a button press, it moves the motor
 * 200 steps in the current direction and then toggles the direction for the next press.
 */
void ControlMotorWithButton(void) 
{
    static uint8_t direction = L6470_DIR_FWD_ID;  // Start in forward direction
    uint8_t  MovementPerRevolution;
    uint32_t Step;
    static uint8_t lastButtonState = GPIO_PIN_RESET;  // Store the last state of the button

    StepperMotorBoardHandle_t *StepperMotorBoardHandle;
    MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;

    // Initialize the motor as per the original example
    MotorParameterDataGlobal = GetMotorParameterInitData();
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(0));
    MotorParameterDataSingle = MotorParameterDataGlobal;
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
     

    MovementPerRevolution = MPR_1;
    // Main program loop
    while (1) {
        uint8_t currentButtonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);  // Read the current state of the button

        if (currentButtonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET) {  // Check for rising edge
            // Move the motor
            //Step =  25600;
            Step = ((uint32_t)MotorParameterDataSingle->fullstepsperrevolution * usrPow(2, MotorParameterDataSingle->step_sel)); // / MovementPerRevolution;
           //StepperMotorBoardHandle->Command->Move(0, 1, direction, MOTOR_STEPS);
            StepperMotorBoardHandle->Command->Move(0, 1, direction, Step);
            while (StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, 0, BUSY_ID) == 0);
            HAL_Delay(DELAY_BETWEEN_MOVEMENTS);

            // Toggle direction for the next button press
            direction = (direction == L6470_DIR_FWD_ID) ? L6470_DIR_REV_ID : L6470_DIR_FWD_ID;
        }

        lastButtonState = currentButtonState;  // Update the last state of the button
        HAL_Delay(10);  // Short delay for debouncing and reducing CPU load
    }
}




void isButtonPressed(void)
{
    static uint8_t lastButtonState = GPIO_PIN_SET; // save last state
    uint8_t currentState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    if (currentState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
      // Home position  
      //BSP_L6470_GoHome(uint8_t ExpBrd_Id, uint8_t L6470_Id)
      BSP_L6470_GoHome(0, 0);
      BSP_L6470_GoHome(0, 1);

    }

    lastButtonState = currentState;  // Update the last state of the button
    HAL_Delay(10);  // Short delay for debouncing and reducing CPU load
}





void USART_DisplayHelpMenu(void) {
    const char *helpMenu = 
    "Quick and Dirty Application for Stepper Motor\r\n"
    "Writer: Younès Ghanem\r\n"
    "\r\n"
    "\r\n"
    "HELP MENU: Motor Control Commands\r\n"
    "---------------------------------\r\n"
    "Command format: [MotorID].[Command].[Direction].[Value]\r\n"
    "\n"
    "Where,\r\n"
    "  MotorID: M0, M1 - Identifier for motors connected to the controller.\r\n"
    "  Command: RUN, MOV, GOTO, SOFTSTOP, HARDSTOP etc. - Action to be performed by the motor.\r\n"
    "  Direction: FWD (Forward), REV (Reverse) - Direction for motion commands.\r\n"
    "  Value: Numeric value, depends on the command (e.g., speed, position).\r\n"
    "\n"
    "Examples:\r\n"
    "  M1.RUN.FWD.200   - Runs motor M1 forward at speed 200.\r\n"
    "  M1.MOV.REV.500   - Moves motor M2 in reverse by 500 steps.\r\n"
    "  M1.GOTO.FWD.1000 - Commands motor M1 to go to position 1000 in forward direction.\r\n"
    "  M0.STOP          - Stops motor M3 immediately.\r\n"
    "\n"
    "Note: Ensure to consult the motor driver datasheet for specific command limitations.\r\n";

    USART_Transmit(&huart2, (uint8_t *)helpMenu);  // Replace 'huart2' with your configured USART handler
}




/**
  * @}
  */ /* End of ExampleExportedFunctions */

/**
  * @brief  This function return the ADC conversion result about the ST1S14 PGOOD.
  * @retval PGOOD The number into the range [0, 4095] as [0, 3.3]V.
  * @note   It will just return 0 if USE_ST1S14_PGOOD is not defined.
  */
uint16_t BSP_ST1S14_PGOOD(void)
{
#ifdef USE_ST1S14_PGOOD
  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  
  return HAL_ADC_GetValue(&HADC);
#else
  return 0;
#endif
}

/**
  * @brief  Calculates the power of a number.
  * @param  base      the base
  * @param  exponent  the exponent
  * @retval power     the result as (base^exponent)
  * @note   There is not OVF control.
  */
uint32_t usrPow(uint8_t base, uint8_t exponent)
{
  uint8_t i;
  uint32_t power = 1;
  
  for (i=0; i<exponent; i++)
    power *= base;
  
  return power;
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case GPIO_PIN_13:
    BSP_EmergencyStop();
    break;
  case L6470_nBUSY_SYNC_GPIO_PIN:
    BSP_L6470_BusySynchEventManager();
    break;
  case L6470_nFLAG_GPIO_PIN:
    BSP_L6470_FlagEventManager();
    break;
  }
}

/**
  * @}
  */ /* End of ExamplePrivateFunctions */

/**
  * @}
  */ /* End of Example */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
