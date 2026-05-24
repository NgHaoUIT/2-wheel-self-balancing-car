/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (FIXED VERSION)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>      // Thêm thư viện này để dùng hàm abs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double previous_error;
    double integral;
    double output_limit;
} PID_Controller_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// [SỬA 1] Định nghĩa chu kỳ vòng lặp cố định
#define LOOP_TIME_MS 5
// [SỬA 3] Deadzone: Giá trị PWM tối thiểu để bánh xe bắt đầu quay (cần chỉnh tùy motor)
#define MOTOR_DEADZONE_LEFT 175
#define MOTOR_DEADZONE_RIGHT 100
#define thresh 0.5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050_Data;
uint32_t timer_main = 0;

PID_Controller_t Balance_PID;
PID_Controller_t Speed_PID_Left;
PID_Controller_t Speed_PID_Right;

int32_t Encoder_Count_A = 0;
int32_t Encoder_Count_B = 0;
double Target_Angle = 0.1;
uint32_t usb_timer = 0;
char usb_buffer[64];

// [SỬA 2] Thêm biến lưu tốc độ đã lọc
double speed_left_filtered = 0;
double speed_right_filtered = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
double calculate_PID(PID_Controller_t *pid, double setpoint, double current_value, double dt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MotorA_SetDir(int dir)
{
    if (dir > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    } else if (dir < 0) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}

void MotorA_SetSpeed(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}
void MotorB_SetDir(int dir)
{
    if (dir > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    } else if (dir < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

void MotorB_SetSpeed(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  while (MPU6050_Init(&hi2c1) == 1);
  timer_main = HAL_GetTick();

  // PID Tuning (Bạn có thể giữ nguyên hoặc chỉnh lại sau)
  Balance_PID.Kp = 315.0; // Tăng Kp một chút vì vòng lặp ổn định hơn
  Balance_PID.Ki = 0.1;
  Balance_PID.Kd = 1.4;
  Balance_PID.output_limit = 999.0;
  Balance_PID.integral = 0;
  Balance_PID.previous_error = 0;

  Speed_PID_Left.Kp = 2.0;
  Speed_PID_Left.Ki = 0.1;
  Speed_PID_Left.Kd = 0.0; // Thường tốc độ không cần D
  Speed_PID_Left.output_limit = 999.0;
  Speed_PID_Left.integral = 0;
  Speed_PID_Left.previous_error = 0;

  Speed_PID_Right.Kp = 2.0;
  Speed_PID_Right.Ki = 0.1;
  Speed_PID_Right.Kd = 0.0;
  Speed_PID_Right.output_limit = 999.0;
  Speed_PID_Right.integral = 0;
  Speed_PID_Right.previous_error = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // [SỬA 1] KIỂM SOÁT THỜI GIAN: Chỉ chạy khi đủ 10ms

	  if (HAL_GetTick() - timer_main < LOOP_TIME_MS) {
		  continue;
      }
      timer_main = HAL_GetTick();
      double dt = (double)LOOP_TIME_MS / 1000.0; // dt cố định

      // 2. Đọc cảm biến
      MPU6050_Read_All(&hi2c1, &MPU6050_Data);
      double current_angle = MPU6050_Data.KalmanAngleY;

      // 3. USB Debug (Gửi góc lên máy tính)
      if (HAL_GetTick() - usb_timer > 5)
      {
           float val = current_angle;
           char sign = (val < 0) ? '-' : '+';
           int int_part = (int)fabs(val);
           int frac_part = (int)((fabs(val) - int_part) * 100);
           int len = sprintf(usb_buffer, "Angle: %c%d.%02d\r\n", sign, int_part, frac_part);
           CDC_Transmit_FS((uint8_t*)usb_buffer, len);
           usb_timer = HAL_GetTick();
      }

    // Safety Cutoff
    if (fabs(current_angle) > 90.0) // Tăng lên 40 độ
    {
        MotorA_SetDir(0); MotorB_SetDir(0);
        MotorA_SetSpeed(0); MotorB_SetSpeed(0);
        Balance_PID.integral = 0;
        Speed_PID_Left.integral = 0; Speed_PID_Right.integral = 0;
        continue;
    }

    double target_speed = calculate_PID(&Balance_PID, Target_Angle, current_angle, dt);

    // ========== VÒNG TRONG: PID TỐC ĐỘ ==========
    int32_t encoder_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t encoder_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

    // Tính tốc độ thô
    double speed_left_raw =  -1.0 * (double)(encoder_left - Encoder_Count_A) / dt;
    double speed_right_raw = (double)(encoder_right - Encoder_Count_B) / dt;

    Encoder_Count_A = encoder_left;
    Encoder_Count_B = encoder_right;

    // [SỬA 2] BỘ LỌC THÔNG THẤP (Low Pass Filter) - Tránh giật do nhiễu encoder
    speed_left_filtered = 0.7 * speed_left_filtered + 0.3 * speed_left_raw;
    speed_right_filtered = 0.7 * speed_right_filtered + 0.3 * speed_right_raw;

    sprintf(usb_buffer, "L: %d | R: %d\r\n", (int)speed_left_filtered, (int)speed_right_filtered);
    CDC_Transmit_FS((uint8_t*)usb_buffer, strlen(usb_buffer));
    // Tính PID Tốc độ dùng giá trị đã lọc
    double desired_speed = 0;
    double speed_correction_left = calculate_PID(&Speed_PID_Left, desired_speed, speed_left_filtered, dt);
    double speed_correction_right = calculate_PID(&Speed_PID_Right, desired_speed, speed_right_filtered, dt);

    // ========== ÁP DỤNG MOTOR ==========
    double final_output_left = 999;
    double final_output_right = target_speed  ;

    // [SỬA ĐOẠN NÀY] XỬ LÝ DEADZONE THÔNG MINH HƠN

      // Motor Trái
      int pwm_left = 0;
      int dir_left = 0;

      // Chỉ bù ma sát khi lực điều khiển đủ lớn (lớn hơn 5)
      // Giúp robot im lặng khi sai số quá nhỏ
      if (final_output_left > thresh) {
          dir_left = -1;
          pwm_left = (int)(final_output_left + MOTOR_DEADZONE_LEFT);
      }
      else if (final_output_left < -thresh) {
          dir_left = 1;
          pwm_left = (int)(fabs(final_output_left) + MOTOR_DEADZONE_LEFT);
      }
      else {
          // Nếu PID tính ra quá nhỏ (< 5), coi như đứng yên, không bơm thêm Deadzone
          dir_left = 0;
          pwm_left = 0;
      }

      // Motor Phải (Làm tương tự)
      int pwm_right = 0;
      int dir_right = 0;

      if (final_output_right > thresh) {
          dir_right = -1;
          pwm_right = (int)(final_output_right + MOTOR_DEADZONE_RIGHT);
      }
      else if (final_output_right < -thresh) {
          dir_right = 1;
          pwm_right = (int)(fabs(final_output_right) + MOTOR_DEADZONE_RIGHT);
      }
      else {
          dir_right = 0;
          pwm_right = 0;
      }

      // Giới hạn Max PWM
      if (pwm_left > 999) pwm_left = 999;
      if (pwm_right > 999) pwm_right = 999;

      // Xuất lệnh ra motor
      MotorA_SetDir(dir_left);
      MotorA_SetSpeed(pwm_left);

      MotorB_SetDir(dir_right);
      MotorB_SetSpeed(pwm_right);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Hàm tính toán đầu ra PID
double calculate_PID(PID_Controller_t *pid, double setpoint, double current_value, double dt)
{
    // 1. Tính toán Sai số (Error)
    double error = setpoint - current_value;

    // 2. Tỷ lệ (P)
    double proportional = pid->Kp * error;

    // 3. Tích phân (I)
    pid->integral += error * dt;
    // Chống bão hòa tích phân (Anti-windup) - Rất quan trọng
    if (pid->Ki != 0) // Chỉ thực hiện khi Ki khác 0
    {
        double limit = pid->output_limit / pid->Ki;

        if (pid->integral > limit) pid->integral = limit;
        if (pid->integral < -limit) pid->integral = -limit;
    }
    else
    {
        // Nếu Ki = 0, tốt nhất nên reset tích phân để tránh nó tích lũy ngầm
        pid->integral = 0;
    }
    // 4. Đạo hàm (D)
    // dt là khoảng thời gian giữa các lần tính toán
    double derivative = (error - pid->previous_error) / dt;

    // 5. Cập nhật lỗi trước đó
    pid->previous_error = error;

    // 6. Đầu ra PID
    double output = proportional + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 7. Giới hạn đầu ra
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

