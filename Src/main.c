/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uartRingBuffer.h"
#include "NMEA.h"
#include "stdio.h"
#include "string.h"
#include "MPU6050.h"
#include "LoRa.h"
#include "Math.h"
#include "ds18b20.h"
#include "sht3x.h"
#include "bmp180_for_stm32_hal.h"
#include "ow.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  A3F    208     // 208 Hz
#define  B3F    233     // 233 Hz
#define  B3     247     // 247 Hz
#define  C4     261     // 261 Hz (middle C)
#define  C4S    277     // 277 Hz
#define  E4F    311     // 311 Hz
#define  F4     349     // 349 Hz
#define  A4F    415     // 415 Hz
#define  B4F    466     // 466 Hz
#define  B4     493     // 493 Hz
#define  C5     523     // 523 Hz
#define  C5S    554     // 554 Hz
#define  E5F    622     // 622 Hz
#define  F5     698     // 698 Hz
#define  F5S    740     // 740 Hz
#define  A5F    831     // 831 Hz

#define  REST   (-1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
LoRa myLoRa;
ds18b20_t ds18;
GPSSTRUCT gpsData;
sht3x_handle_t sht3x;

char msg[] = "\r\nTransmit Success!\r\n";
char msg0[] = "\r\nTransmit Failed!\r\n";

char GGA[100];
char RMC[100];


uint32_t t_mpu = 0;
uint32_t t_gps = 0;
uint32_t t_lora = 0;
uint32_t t_temp = 0;

int flagGGA = 0, flagRMC = 0;
char Buffer [160];


int VCCTimeout = 5000; // GGA or RMC will not be received if the VCC is not sufficient

float tempDUM = 0;
float humiDUM = 0;
float presDUM = 1008.0;
int	  piezDUM = 0;
int   stampDUM = 0;


/* Tempo & jeda antar nada (mengikuti Arduino) */
static const uint16_t RICK_BEAT_LEN_MS = 100;
static const float    RICK_SEP_CONST   = 0.3f;

/* ===== Chorus ===== */
static const int16_t song1_chorus_melody[] =
{
    B4F, B4F, A4F, A4F,
    F5, F5, E5F, B4F, B4F, A4F, A4F, E5F, E5F, C5S, C5, B4F,
    C5S, C5S, C5S, C5S,
    C5S, E5F, C5, B4F, A4F, A4F, A4F, E5F, C5S,
};

static const uint8_t song1_chorus_rhythm[] =
{
    1, 1, 1, 1,
    3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
    1, 1, 1, 1,
    3, 3, 3, 1, 2, 2, 2, 4, 8,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*	######## Settingan Temperatur	###########*/
float DS18_ReadTemperature(ds18b20_t *dev)
{
    // Mulai konversi
    ds18b20_cnv(dev);
    while (ds18b20_is_busy(dev));          // tunggu bus
    while (!ds18b20_is_cnv_done(dev));     // tunggu selesai konversi

    // Minta data scratchpad
    ds18b20_req_read(dev, 0);
    while (ds18b20_is_busy(dev));

    // Ambil hasil
    int16_t raw = ds18b20_read_c(dev);     // °C x 100


    if (raw == DS18B20_ERROR)
    {
        printf("DS18 read error, last_err=%d\r\n",
               (int)ds18b20_last_error(dev));
        return -1000.0f;
    }

    return raw / 100.0f;  // konversi ke float Celsius
}

void ds18_tim_cb(TIM_HandleTypeDef *htim)
{
    ow_callback(&ds18.ow);
}



/* ===== Definisi nada & array Rickroll sama seperti punyamu ===== */

/* Helper untuk hitung jumlah elemen array */
#define ARR_SIZE(x)  (sizeof(x) / sizeof((x)[0]))

/* 1. LETAKKAN FUNGSI-FUNGSI DASAR BUZZER DI ATAS */
static void Buzzer_SetDuty(uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t cmp = (arr + 1) * duty_percent / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
}

static void Buzzer_SetFreq(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        Buzzer_SetDuty(0);
        return;
    }

    // Asumsi: TIM1 tick = 1 MHz (PSC diset di CubeMX)
    uint32_t tim_tick_hz = 1000000U;
    uint32_t arr = (tim_tick_hz / freq_hz) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);

    uint32_t cmp = (arr + 1) / 2;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
}

static void Buzzer_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    Buzzer_SetDuty(0);
}

static void Buzzer_PlayTone(uint16_t freq_hz, uint16_t dur_ms)
{
    if (freq_hz == 0) {
        Buzzer_SetDuty(0);
        HAL_Delay(dur_ms);
        return;
    }

    Buzzer_SetFreq(freq_hz);
    Buzzer_SetDuty(50);
    HAL_Delay(dur_ms);
    Buzzer_SetDuty(0);
    HAL_Delay(20);
}

static void Buzzer_Beep_OK(void)
{
    Buzzer_PlayTone(2000, 80);
}

static void Buzzer_Beep_ERR(void)
{
    Buzzer_PlayTone(500, 1000);
}

static void Buzzer_PlayMeme_PleaseSpeed(void)
{
    // Tone seperti meme "please speed I need this"
    // Serangkaian beep cepat dengan pitch naik-turun

    const uint16_t notes[] = {
        900, 1200, 1000, 1500, 800,
        1400, 1600, 1800, 1300, 2000
    };

    const uint16_t durations[] = {
        80, 70, 60, 70, 50,
        60, 60, 70, 50, 120
    };

    uint8_t count = sizeof(notes)/sizeof(notes[0]);

    for (uint8_t i = 0; i < count; i++)
    {
        Buzzer_PlayTone(notes[i], durations[i]);
    }

    // Final "panic" trill
    for (int i = 0; i < 6; i++)
    {
        Buzzer_PlayTone(2000 + (i * 200), 40);
    }

    // Tutup dengan beep panjang
    Buzzer_PlayTone(800, 200);
}


static void Buzzer_PlayToneRick(int16_t freq_hz, uint16_t dur_ms, float sep)
{
    uint32_t sep_ms = (uint32_t)(dur_ms * sep);

    if (freq_hz > 0) {
        Buzzer_SetFreq((uint32_t)freq_hz);
        Buzzer_SetDuty(50);
        HAL_Delay(dur_ms);
        Buzzer_SetDuty(0);
    } else {
        Buzzer_SetDuty(0);
        HAL_Delay(dur_ms);
    }

    HAL_Delay(sep_ms);
}

static void Buzzer_PlayRickroll_Full(void)
{
    uint32_t n;
    uint16_t len;

    // Chorus
    n = ARR_SIZE(song1_chorus_melody);
    for (uint32_t i = 0; i < n; i++) {
        len = RICK_BEAT_LEN_MS * song1_chorus_rhythm[i];
        Buzzer_PlayToneRick(song1_chorus_melody[i], len, RICK_SEP_CONST);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  uint32_t t_vcc  = HAL_GetTick();

  myLoRa = newLoRa();
  myLoRa.hSPIx      = &hspi1;
  myLoRa.CS_port    = NSS_GPIO_Port;  myLoRa.CS_pin    = NSS_Pin;
  myLoRa.reset_port = RST_GPIO_Port;  myLoRa.reset_pin = RST_Pin;
  myLoRa.DIO0_port  = DIO0_GPIO_Port; myLoRa.DIO0_pin  = DIO0_Pin;

  myLoRa.frequency             = 433;          // Standarnya gini
  myLoRa.spredingFactor        = SF_8;
  myLoRa.bandWidth             = BW_125KHz;
  myLoRa.crcRate               = CR_4_6;
  myLoRa.preamble              = 8;
  myLoRa.power                 = POWER_20db;
  myLoRa.overCurrentProtection = 100;

  LoRa_setSyncWord(&myLoRa, 0x12);  // Samain ama TX janlup

  if (LoRa_init(&myLoRa) == LORA_OK)
      printf("LoRa RX initialized @%d MHz, SF7/BW125/CR4_6 SW=0x12\r\n", myLoRa.frequency);

  else
      printf("LoRa init FAIL\r\n");

  Ringbuf_init();

  MPU6050_Initialization();
  MPU6050_Calibrate(&MPU6050, 1500, 2);
  MPU6050_InitAnglesFromAccel(&MPU6050);

  sht3x.i2c_handle = &hi2c1;
  // Jika pin ADDR ke GND:
  sht3x.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW;
  // Kalau ADDR ke VCC ganti ke *_ADDR_PIN_HIGH

  if (!sht3x_init(&sht3x)) {
      printf("SHT3x access failed.\r\n");
  } else {
      printf("SHT3x init OK.\r\n");
  }

  // ==== Init BMP180 (GY-87) ====
  BMP180_Init(&hi2c1);                          // pakai I2C1 yang sama dengan MPU6050
  BMP180_SetOversampling(BMP180_STANDARD);      // mode oversampling standar
  BMP180_UpdateCalibrationData();               // WAJIB: baca data kalibrasi dari EEPROM sensor
  // ==============================

  //	Onewire sama ds18b20

    ow_init_t ow_init_struct;
    ow_init_struct.tim_handle = &htim3;
    ow_init_struct.gpio = GPIOB;
    ow_init_struct.pin = GPIO_PIN_8;
    ow_init_struct.tim_cb = ds18_tim_cb;
    ow_init_struct.done_cb = NULL;
    ow_init_struct.rom_id_filter = 0;

    ds18b20_init(&ds18, &ow_init_struct);

    // Update ROM IDs for all devices
    ds18b20_update_rom_id(&ds18);
    while(ds18b20_is_busy(&ds18));

    // Configure alarm thresholds and resolution
    ds18b20_config_t ds18_conf = {
        .alarm_high = 50,
        .alarm_low = -50,
        .cnv_bit = DS18B20_CNV_BIT_12
    };
    ds18b20_conf(&ds18, &ds18_conf);
    while(ds18b20_is_busy(&ds18));

  Buzzer_Init();              // start PWM TIM1 CH1
  Buzzer_PlayMeme_PleaseSpeed();
//  Buzzer_PlayHappyBirthday(); // putar melody di startup
//  Buzzer_PlayMeme_PleaseSpeed();
  HAL_Delay (500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now = HAL_GetTick(); // Tick Scheduling ea

	  if(now - t_mpu >= 20){
		  MPU6050_Update(&MPU6050); // Update MPU tiap 20ms || Overkill?
		  t_mpu = now;
	  }

	  if(now - t_temp >= 100){
		  tempDUM = DS18_ReadTemperature(&ds18);	// Read dan update temperature (udah /100)

          // Baca suhu + kelembaban dari SHT3x
          float t, h;
          sht3x_read_temperature_and_humidity(&sht3x, &t, &h);

          // Pakai hasil SHT3x untuk payload
          humiDUM = h;
//          tempDUM = t;

          // Tekanan dari BMP180 (GY-87)
          int32_t bmpPresPa = BMP180_GetPressure();   // satuan Pa
          presDUM = bmpPresPa / 100.0f;               // konversi ke hPa (mbar)

          t_temp = now;


	  }

	  /* GPS Parser (JANGAN diotak atik) */
	  if(Wait_for("GGA") == 1){
		  VCCTimeout = 5000;
		  Copy_upto("*", GGA);
		  flagGGA = (decodeGGA(GGA, &gpsData.ggastruct) == 0 )? 2 : 1;
	  }

	  if(Wait_for("RMC") == 1){
		  VCCTimeout = 5000;
		  Copy_upto("*", RMC);
		  flagRMC = (decodeRMC(RMC, &gpsData.rmcstruct) == 0 )? 2 : 1;
	  }

	  if(now - t_vcc >= 1000){
		  if(VCCTimeout > 0) VCCTimeout -= 1000; // Timeout VCC GPS
		  t_vcc = now;
	  }

	  if(now - t_lora >= 100){ // Transmit tiap 500ms
		  snprintf(Buffer, sizeof(Buffer),
				  "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.10f,%.10f,%.2f,%d,%.2f,%.2f,%d",
		  	  	  // Temp, Humi, pres  accx	 accy  accz  gyrx  gyry  gyrz  pitc  roll  head  LATI	LONG   ALTI  SATC  HDOP  SPD PIEZ
				  stampDUM,
				  tempDUM,
				  humiDUM,
				  presDUM,
				  MPU6050.acc_x_f,
				  MPU6050.acc_y_f,
				  MPU6050.acc_z_f,
				  MPU6050.gyro_x_f,
				  MPU6050.gyro_y_f,
				  MPU6050.gyro_z_f,
				  MPU6050.pitch_deg,
				  MPU6050.roll_deg,
				  MPU6050.yaw_deg,
				  gpsData.ggastruct.lcation.latitude,
				  gpsData.ggastruct.lcation.longitude,
				  gpsData.ggastruct.alt.altitude,
				  gpsData.ggastruct.numofsat,
				  gpsData.ggastruct.hdop,
				  gpsData.rmcstruct.speed,
				  piezDUM
				  );

		  uint8_t status = LoRa_transmit(&myLoRa, (uint8_t*)Buffer, strlen(Buffer), 2000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, strlen(Buffer), HAL_MAX_DELAY);

		  if(status == 1){
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
			  Buzzer_Beep_OK();
		  }
		  else{
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg0, strlen(msg0), 100);
			  Buzzer_Beep_ERR();
		  }
		  stampDUM++;
		  if(stampDUM > 9999) stampDUM = 0;
		  t_lora = now;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 78;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Timer yang dipakai untuk OneWire = TIM3
    if (htim->Instance == TIM3)
    {
        ds18_tim_cb(htim);   // ini akan memanggil ow_callback(&ds18.ow);
    }
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
