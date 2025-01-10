/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "BMPXX80.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RX_Buffer[10];
uint8_t RX_Char;
uint8_t buffer_index = 0;
char Buff[16];

float temperature = 0;
float pressure = 0;
uint16_t Output = 0;

uint16_t CNT = 0;

float Error = 0.0;
float OUTPUT = 0.0;
float Temperatura = 0.0;
uint8_t Ds_OK = 1;
volatile uint8_t iRQ = 0;
uint8_t rom_code[8];
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float previous_error;
    float integral;
    float setpoint;

    float output_min;
    float output_max;
} PID_Controller;

PID_Controller PID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/*------------------------------------------------PID-----------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_min, float output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->setpoint = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;
}
float PID_Compute(PID_Controller *pid, float measured_value, float dt) {
    float error = pid->setpoint - measured_value;
    float derivative = (error - pid->previous_error) / dt;
    pid->integral += error * dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if(output > pid->output_max){
        output = pid->output_max;
    }else if(output < pid->output_min){
        output = pid->output_min;
    }

    pid->previous_error = error;

    Error = error;
    OUTPUT = output;

    return output;
}
void SetPWM(uint16_t PWM){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}


void Uart_Send_String(char *d){
    HAL_UART_Transmit(&huart1, (uint8_t *)d, strlen(d), 2000);
}

uint32_t Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}
void delay_us(uint32_t us)
{
	uint32_t au32_initial_ticks = DWT->CYCCNT;
	uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
	us *= au32_ticks;
	while ((DWT->CYCCNT - au32_initial_ticks) < us-au32_ticks);
}
HAL_StatusTypeDef wire_reset(void){
  int rc;

  DS_GPIO_Port -> ODR &= ~DS_Pin;
  delay_us(480);
  DS_GPIO_Port -> ODR |= DS_Pin;
  delay_us(70);
  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
  delay_us(410);

  if (rc == 0)
    return HAL_OK;
  else
    return HAL_ERROR;
}
void write_bit(int value)
{
  if (value) {
	DS_GPIO_Port -> ODR &= ~DS_Pin;
    delay_us(6);
    DS_GPIO_Port -> ODR |= DS_Pin;
    delay_us(64);
  } else {
	DS_GPIO_Port -> ODR &= ~DS_Pin;
    delay_us(60);
    DS_GPIO_Port -> ODR |= DS_Pin;
    delay_us(10);
  }
}
int read_bit(void)
{
  int rc;
  DS_GPIO_Port -> ODR &= ~DS_Pin;
  delay_us(6);
  DS_GPIO_Port -> ODR |= DS_Pin;
  delay_us(9);
  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
  delay_us(55);
  return rc;
}
void wire_write(uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    write_bit(byte & 0x01);
    byte >>= 1;
  }
}
uint8_t wire_read(void)
{
  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit())
      value |= 0x80;
  }
  return value;
}
HAL_StatusTypeDef DS_SetResolution(uint8_t resolution) {
    uint8_t scratchpad[9];
    uint8_t config;
    if (resolution < 9 || resolution > 12) {
        return HAL_ERROR; // Niepoprawna rozdzielczość
    }

    // Reset i inicjalizacja
    if (wire_reset() != HAL_OK) {
        return HAL_ERROR;
    }

    // Wybór urządzenia (SKIP ROM)
    wire_write(0xCC);

    // Odczyt Scratchpada
    wire_write(0xBE);
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = wire_read();
    }

    // Modyfikacja rejestru konfiguracji
    config = scratchpad[4];
    config &= ~0x60; // Wyczyszczenie bitów R1 i R0
    config |= (resolution - 9) << 5; // Ustawienie odpowiednich bitów

    // Zapis nowego Scratchpada
    if (wire_reset() != HAL_OK) {
        return HAL_ERROR;
    }

    wire_write(0xCC); // SKIP ROM
    wire_write(0x4E); // Write Scratchpad
    wire_write(scratchpad[2]); // Kopiowanie TH
    wire_write(scratchpad[3]); // Kopiowanie TL
    wire_write(config);        // Nowy rejestr konfiguracji

    // Zapis do EEPROM (kopiowanie Scratchpada do pamięci)
    if (wire_reset() != HAL_OK) {
        return HAL_ERROR;
    }

    wire_write(0xCC); // SKIP ROM
    wire_write(0x48); // Kopiowanie Scratchpada do EEPROM
    HAL_Delay(10);    // Krótka przerwa na zapis

    return HAL_OK;
}
float ReadTemperature(void) {
    uint8_t temp_lsb, temp_msb;
    int16_t temp_raw;
    float temperature;

    // Reset i inicjalizacja czujnika
    if (wire_reset() != HAL_OK) {
        return -999.0; // Błąd komunikacji
    }

    // Wybór urządzenia (SKIP ROM - pomijamy adresowanie, zakładając jeden czujnik)
    wire_write(0xCC);

    // Wydanie polecenia rozpoczęcia konwersji temperatury
    wire_write(0x44);

    // Czekanie na zakończenie konwersji (czas max. 750 ms)
    HAL_Delay(100);

    // Reset i ponowne inicjalizowanie do odczytu danych
    if (wire_reset() != HAL_OK) {
        return -999.0; // Błąd komunikacji
    }

    // Wybór urządzenia (SKIP ROM)
    wire_write(0xCC);

    // Wydanie polecenia odczytu Scratchpad
    wire_write(0xBE);

    // Odczyt dwóch pierwszych bajtów Scratchpad (temp LSB i MSB)
    temp_lsb = wire_read();
    temp_msb = wire_read();

    // Złożenie surowej wartości temperatury
    temp_raw = (temp_msb << 8) | temp_lsb;

    // Przekształcenie na wartość temperatury w stopniach Celsjusza
    temperature = temp_raw / 16.0;

    return temperature;
}

void SendDataToPC() {
    int8_t MSB = 0;
    uint8_t LSB = 0; // LSB jako uint8_t dla poprawności zakresu
    char Buff[10];   // Bufor do konwersji liczb na ciągi znaków

    // Temperaturę wysyłamy jako "<MSB.LSB/"
    MSB = (int8_t)temperature;
    LSB = (uint8_t)((temperature - MSB) * 100.0);
    Uart_Send_String("<");
    itoa(MSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String(".");
    itoa(LSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String("/");

    // Błąd (Error)
    MSB = (int8_t)Error;
    LSB = (uint8_t)(fabs(Error * 100.0) - abs(MSB * 100)); // fabs dla liczb zmiennoprzecinkowych
    itoa(MSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String(".");
    itoa(LSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String("/");

    // Setpoint (PID)
    MSB = (int8_t)PID.setpoint;
    LSB = (uint8_t)((PID.setpoint - MSB) * 100.0);
    itoa(MSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String(".");
    itoa(LSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String("/");

    // Temperatura
    MSB = (int8_t)Temperatura;
    LSB = (uint8_t)((Temperatura - MSB) * 100.0);
    itoa(MSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String(".");
    itoa(LSB, Buff, 10);
    Uart_Send_String(Buff);
    Uart_Send_String(">\r\n");
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3){
		iRQ = 1;
		CNT++;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
		char *endptr;
		float temp = 0.0;

		if(buffer_index < 10) {
		     RX_Buffer[buffer_index++] = RX_Char;
		}else{
             buffer_index = 0;
		}

		if((RX_Buffer[0] == '<') && (RX_Char == '>')){  //Koniec linii
			if (RX_Buffer[1] == '#') {
			    // KP                                                                         //Kp - <#...>
			    temp = strtof((char*)&RX_Buffer[2], &endptr);
			    if (endptr != (char*)&RX_Buffer[2]) {
			        PID.Kp = temp;
			    }

			    //utoa((uint16_t)PID.Kp, Buff, 10);
			    //Uart_Send_String("KP ustawiono na: ");
			   // Uart_Send_String(Buff);
			   // Uart_Send_String("\r\n");
			}

			if (RX_Buffer[1] == '$') {
			    // KI                                                                         //Ki - <$...>
			    temp = strtof((char*)&RX_Buffer[2], &endptr);
			    if (endptr != (char*)&RX_Buffer[2]) {
			        PID.Ki = temp;
			    }

			    //utoa((uint16_t)PID.Ki, Buff, 10);
			 	//Uart_Send_String("KI ustawiono na: ");
			 	//Uart_Send_String(Buff);
			 	//Uart_Send_String("\r\n");
			}

			if (RX_Buffer[1] == '^') {
			    // KD                                                                          //Kd - <^...>
			    temp = strtof((char*)&RX_Buffer[2], &endptr);
			    if (endptr != (char*)&RX_Buffer[2]) {
			        PID.Kd = temp;
			    }

			    //utoa((uint16_t)PID.Kd, Buff, 10);
			 	//Uart_Send_String("KD ustawiono na: ");
			 	//Uart_Send_String(Buff);
			 	//Uart_Send_String("\r\n");
			}

			if (RX_Buffer[1] == '*') {
			    // Setpoint                                                                //Setpoint - <*...>
			    temp = strtof((char*)&RX_Buffer[2], &endptr);
			    if (endptr != (char*)&RX_Buffer[2]) {
			        PID.setpoint = temp;
			    }

			    //utoa((uint16_t)PID.setpoint, Buff, 10);
			 	//Uart_Send_String("Setpoint ustawiono na: ");
			 	//Uart_Send_String(Buff);
			 	//Uart_Send_String("\r\n");
			}
			for(uint8_t i = 0; i < 10; i++){
				RX_Buffer[i] = 0;
			}
			buffer_index = 0;
		}
		HAL_UART_Receive_IT(&huart1, &RX_Char, 1);
	}
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, &RX_Char, 1);
  HAL_TIM_Base_Start_IT(&htim3);


  Delay_Init();
  PID_Init(&PID, 27.0, 0.9, 4.5, 0, 999);  //55.8, 0.05, 7.5
  BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
  PID.setpoint = 0.0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(iRQ == 1){
		temperature = BMP280_ReadTemperature();
		Output = PID_Compute(&PID, temperature, 0.02);
		SetPWM(Output);
        iRQ = 0;
	}

	if(CNT == 3){
		Temperatura = ReadTemperature();
		SendDataToPC();
		CNT = 0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
