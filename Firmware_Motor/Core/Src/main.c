#include "main.h"
#include "motor.h"
#include "speed.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private function prototypes */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM8_Init(void);
void MX_TIM2_Init(void);
void MX_USART3_UART_Init(void);
void UART_Process(void);
void UART_ProcessCommand(const char *cmd);

/* Global Peripheral Handlers */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart3;

/* UART State variables */
#define UART_RX_BUF_SIZE 64
static uint8_t rx_buffer[UART_RX_BUF_SIZE];
static uint8_t rx_byte;
static uint16_t rx_index = 0;
static volatile uint8_t command_ready = 0;

/* Hall sensor validation state */
static uint8_t previous_hall_state = 0;
static uint8_t hall_error_counter = 0;
#define HALL_ERROR_MAX_LIMIT 3

bool new_speed_data_available = false;
volatile uint32_t last_hall_transition_time = 0;

/* Hall State Helper */
uint8_t Hall_GetState(void)
{
    uint8_t state = 0;
    // Reads PB6, PB7, PB8 corresponding to Hall A, B, C digital inputs
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) state |= 0x01; // Hall A
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) state |= 0x02; // Hall B
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) state |= 0x04; // Hall C
    return state;
}

/**
 * @brief EXTI Callback for Hall sensors & Fault pins
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Hall sensor state change
    if (GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_8)
    {
        uint8_t current_hall = Hall_GetState();
        uint32_t current_ticks = __HAL_TIM_GET_COUNTER(&htim2); // 1us free-running timer counter
        
        last_hall_transition_time = HAL_GetTick(); // Update transition timestamp
        
        // Safety: Invalid Hall State check (000 or 111)
        if (current_hall == 0 || current_hall == 7)
        {
            Motor_Stop();
            UART_SendMessage("FAULT: Invalid Hall State Detected!\r\n");
            return;
        }
        
        // Transition validation
        if (previous_hall_state != 0)
        {
            // Expected forward sequence transitions: 1->5->4->6->2->3->1
            // Expected reverse sequence transitions: 1->3->2->6->4->5->1
            bool valid_transition = false;
            
            switch (previous_hall_state)
            {
                case 1: if (current_hall == 5 || current_hall == 3) valid_transition = true; break;
                case 5: if (current_hall == 4 || current_hall == 1) valid_transition = true; break;
                case 4: if (current_hall == 6 || current_hall == 5) valid_transition = true; break;
                case 6: if (current_hall == 2 || current_hall == 4) valid_transition = true; break;
                case 2: if (current_hall == 3 || current_hall == 6) valid_transition = true; break;
                case 3: if (current_hall == 1 || current_hall == 2) valid_transition = true; break;
                default: break;
            }
            
            if (!valid_transition)
            {
                hall_error_counter++;
                if (hall_error_counter >= HALL_ERROR_MAX_LIMIT)
                {
                    Motor_Stop();
                    UART_SendMessage("FAULT: Persistent Hall Noise / Wiring Error!\r\n");
                    return;
                }
            }
            else
            {
                if (hall_error_counter > 0) hall_error_counter--;
            }
        }
        
        previous_hall_state = current_hall;
        
        // Calculate RPM period and call commutation updates
        Speed_ProcessTransition(current_ticks);
        new_speed_data_available = true;
        
        Motor_Commutate(current_hall);
    }
}

/**
 * @brief USART RX completion callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';
                command_ready = 1;
            }
        }
        else
        {
            if (rx_index < (UART_RX_BUF_SIZE - 1))
            {
                rx_buffer[rx_index++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

void UART_SendMessage(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void UART_Process(void)
{
    if (command_ready)
    {
        UART_ProcessCommand((char *)rx_buffer);
        rx_index = 0;
        command_ready = 0;
    }
}

void UART_ProcessCommand(const char *cmd)
{
    if (strncmp(cmd, "ENABLE", 6) == 0)
    {
        Motor_Enable();
    }
    else if (strncmp(cmd, "DISABLE", 7) == 0)
    {
        Motor_Disable();
    }
    else if (strncmp(cmd, "START", 5) == 0)
    {
        Motor_Start();
    }
    else if (strncmp(cmd, "STOP", 4) == 0)
    {
        Motor_Stop();
        UART_SendMessage("MSG: Motor Stop Request Received\r\n");
    }
    else if (strncmp(cmd, "FORWARD", 7) == 0)
    {
        Motor_SetDirection(MOTOR_DIR_FORWARD);
        UART_SendMessage("MSG: Direction set to FORWARD\r\n");
    }
    else if (strncmp(cmd, "BACKWARD", 8) == 0)
    {
        Motor_SetDirection(MOTOR_DIR_BACKWARD);
        UART_SendMessage("MSG: Direction set to BACKWARD\r\n");
    }
    else if (strncmp(cmd, "SET_SPEED", 9) == 0)
    {
        int speed = 0;
        if (sscanf(cmd, "SET_SPEED %d", &speed) == 1)
        {
            Motor_SetSpeed((uint16_t)speed);
            char resp[32];
            snprintf(resp, sizeof(resp), "MSG: Speed set to %d RPM\r\n", speed);
            UART_SendMessage(resp);
        }
    }
    else if (strncmp(cmd, "SET_PID", 7) == 0)
    {
        float kp, ki, kd;
        if (sscanf(cmd, "SET_PID %f %f %f", &kp, &ki, &kd) == 3)
        {
            Motor_SetPID(kp, ki, kd);
            UART_SendMessage("MSG: PID Gains Updated\r\n");
        }
    }
    else if (strncmp(cmd, "GET_SPEED", 9) == 0)
    {
        char resp[64];
        snprintf(resp, sizeof(resp), "Target: %d RPM | Actual: %d RPM\r\n", 
                 Motor_GetTargetSpeed(), Motor_GetActualSpeed());
        UART_SendMessage(resp);
    }
    else if (strncmp(cmd, "GET_PID", 7) == 0)
    {
        float kp, ki, kd;
        Motor_GetPID(&kp, &ki, &kd);
        char resp[64];
        snprintf(resp, sizeof(resp), "PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", kp, ki, kd);
        UART_SendMessage(resp);
    }
    else if (strncmp(cmd, "STATUS", 6) == 0)
    {
        char resp[128];
        const char *state_str = "UNKNOWN";
        switch (Motor_GetState())
        {
            case MOTOR_STATE_INIT: state_str = "INIT"; break;
            case MOTOR_STATE_READY: state_str = "READY"; break;
            case MOTOR_STATE_IDLE: state_str = "IDLE"; break;
            case MOTOR_STATE_ALIGNING: state_str = "ALIGNING"; break;
            case MOTOR_STATE_RUNNING: state_str = "RUNNING"; break;
            case MOTOR_STATE_FAULT: state_str = "FAULT"; break;
            case MOTOR_STATE_RESET: state_str = "RESET"; break;
        }
        snprintf(resp, sizeof(resp), "STATE: %s | DIR: %d | TARGET: %d RPM | ACTUAL: %d RPM | DUTY: %d%%\r\n",
                 state_str, Motor_GetDirection(), Motor_GetTargetSpeed(), Motor_GetActualSpeed(), Motor_GetPWMDuty());
        UART_SendMessage(resp);
    }
    else if (strncmp(cmd, "RESET", 5) == 0)
    {
        Motor_ResetFault();
    }
    else
    {
        UART_SendMessage("ERR: Unknown command\r\n");
    }
}

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_TIM2_Init();
    MX_USART3_UART_Init();

    /* Initialize motor control state structure */
    Motor_Init();

    /* Start TIM2 free-running counter */
    HAL_TIM_Base_Start(&htim2);
    
    /* Enable UART RX Interrupt */
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    
    UART_SendMessage("STM32G431 Firmware Ready\r\n");

    /* Infinite loop */
    while (1)
    {
        UART_Process();
        Motor_Process();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIO pin Output Level for EN_GATE */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

    /* Configure GPIO pin : PA11 (DRV8302 EN_GATE) */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure GPIO pins : PA0 (nFAULT) and PA1 (nOCTW) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init for Fault inputs */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* Configure GPIO pins : PB6 PB7 PB8 (Hall Sensors) */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI9_5 interrupt init for Hall Sensors */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; // Center-aligned PWM
    htim1.Init.Period = 8500; // 20kHz at 170MHz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Master configuration to trigger TIM8 slave
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1; // Output compare trigger
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Config deadtime configuration
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 170; // 1.0 us at 170MHz
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW; // Active low break input
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period = 8500;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Sync TIM8 to TIM1 Master
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 170; // 1.0 us at 170MHz
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 169; // 170MHz / 170 = 1MHz counter clock (1us ticks)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;  // Full 32-bit range for microsecond timestamp counter
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (tim_pwmHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        
        /**TIM1 GPIO Configuration
        PA8     ------> TIM1_CH1
        PC13    ------> TIM1_CH1N
        PA9     ------> TIM1_CH2
        PB14    ------> TIM1_CH2N
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM1; 
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        // TIM1_BKIN on PA6
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        /**TIM8 GPIO Configuration
        PA15    ------> TIM8_CH1
        PA7     ------> TIM8_CH1N
        PA10    ------> TIM8_BKIN
        */
        GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        // TIM8_BKIN on PA10
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF11_TIM8;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0); // Loop runs at lower priority than EXTI Hall speed logic
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (uartHandle->Instance == USART3)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* USART3 clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}
