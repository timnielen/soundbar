/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "input_detection.h"
#include "pc_control.h"
#include "pd_controller.h"

#include "stm32g0xx_hal_def.h"

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// static void audio_start_aux(void)
// {
//     /* 1. amps out of shutdown (PDN high + 5 ms) */
//     soundbar_dsp_power_on();

//     /* 2. clocks: no Bluetooth board -> the PCM1808 becomes I2S master
//      *    and its oscillator starts. With the ESP32 board fitted, the
//      *    ESP32 would start its clocks here instead.                    */
//     if (HAL_GPIO_ReadPin(BT_PRESENT_GPIO_Port, BT_PRESENT_Pin) ==
//     GPIO_PIN_SET) {
//         HAL_GPIO_WritePin(ADC_MASTER_GPIO_Port, ADC_MASTER_Pin,
//         GPIO_PIN_SET);
//     }
//     HAL_Delay(20);                     /* oscillator + clocks settle      */

//     /* 3. reset, configure, play (both amps) */
//     if (soundbar_dsp_start(48000) != HAL_OK) {
//         /* an amp did not answer on I2C: wrong address, PDN, solder joint */
//         Error_Handler();
//     }
//     soundbar_dsp_set_volume(0.0f);
// }
#define TAS_PDN_PORT PDN__GPIO_Port
#define TAS_PDN_PIN PDN__Pin
static void tas_pdn(uint8_t on) {
  HAL_GPIO_WritePin(TAS_PDN_PORT, TAS_PDN_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void start_adc(void) {
  HAL_GPIO_WritePin(ADC_MASTER_GPIO_Port, ADC_MASTER_Pin, GPIO_PIN_SET);
}

static void stop_adc(void) {
  HAL_GPIO_WritePin(ADC_MASTER_GPIO_Port, ADC_MASTER_Pin, GPIO_PIN_RESET);
}

#define TAS_REG_PAGE 0x00 /* on every page: selects the page     */
#define TAS_REG_BOOK 0x7F /* on page 0 only: selects the book    */

#define TAS_REG_CTRL_1 0x02 /* switching frequency, modulation     */
#define TAS_STATE_768KHZ 0x00
#define TAS_STATE_384KHZ 0x10
#define TAS_STATE_BD 0x00   // in theory no inductor needed i think
#define TAS_STATE_1SPW 0x01 // efficient but requires LC filter
#define TAS_STATE_HYBRID                                                       \
  0x02 // requires the supply voltage to be specified using the PPC3 tool which
       // we dont have access to

#define TAS_REG_CTRL_2                                                         \
  0x03 /* state (deep sleep/sleep/Hi-Z/play), mute, DSP reset */
#define TAS_STATE_DEEP_SLEEP 0x00
#define TAS_STATE_SLEEP 0x01
#define TAS_STATE_HIZ 0x02
#define TAS_STATE_DIS_DSP 0x10
#define TAS_STATE_PLAY 0x03
#define TAS_STATE_MUTE 0x08

#define TAS_REG_SAP_CTRL_3 0x35 /* which input goes to which DSP path  */
#define TAS_STATE_BOTH_RIGHT 0x21
#define TAS_STATE_BOTH_LEFT 0x12

#define TAS_REG_DIG_VOL 0x4C /* master volume, -0.5 dB steps, default 0x30 */

#define TAS_REG_AGAIN 0x54 /* analog gain, default 0x00 */

#define TAS_REG_DAC_CTRL 0x5D /* DAC control, default 0xF8 */

#define TAS_REG_PHASE_CTRL 0x6A /* phase control */
#define TAS_STATE_I2S_SYNC_EN 0x02
#define TAS_STATE_PHASE_SYNC_EN 0x01
#define TAS_PHASE(n) ((uint8_t)((n) << 2))

#define CHAN_FAULT 0x70
#define GLOBAL_FAULT1 0x71
#define GLOBAL_FAULT2 0x72
#define OT_WARNING 0x73

#define TAS_REG_FAULT_CLEAR 0x78
#define TAS_FAULT_CLEAR 0x80

#define TAS_REG_PIN_CONTROL2                                                   \
  0x75 /* fault latch enables [DS Table 7-48], default 0xF8 */
#define TAS_LATCHES_OFF                                                        \
  0xC0 /* bits 7-6 reserved = 11; bits 5-3 = 0: clock fault,                   \
          OTSD and OT warning flags show the current status */
#define TAS_REG_MISC_CONTROL 0x76 /* [DS Table 7-49], default 0x00 */
#define TAS_OTSD_AUTO_REC                                                      \
  0x10 /* bit 4: restart by itself after over-temperature */

#define TAS_MAX_CHAN_FAULTS 3 /* overcurrent/DC retries before giving up */

#define TAS_REG_DSP_MISC 0x66 /* EQ/DRC bypass, separate L/R coefs   */
#define TAS_STATE_LR_SEPERATE 0x08

#define TAS_BOOK_CTRL 0x00 /* control port registers above        */

#define TAS_BOOK_DSP 0x8C
#define TAS_PAGE_INPUT_MIXER 0x29
#define TAS_STATE_L_TO_L 0x18
#define TAS_STATE_R_TO_L 0x1C
#define TAS_STATE_L_TO_R 0x20
#define TAS_STATE_R_TO_R 0x24

#define TAS_I2C (&hi2c2)
#define TAS_I2C_TIMEOUT_MS 10
HAL_StatusTypeDef tas_write(uint8_t addr, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(TAS_I2C, (uint16_t)(addr), reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, TAS_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef tas_write_32(uint8_t addr, uint8_t reg, uint32_t val) {
  uint32_t be_val = __builtin_bswap32(val); // convert to big-endian
  return HAL_I2C_Mem_Write(TAS_I2C, (uint16_t)(addr), reg, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)&be_val, 4, TAS_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef tas_read(uint8_t addr, uint8_t reg, uint8_t *val) {
  return HAL_I2C_Mem_Read(TAS_I2C, (uint16_t)(addr), reg, I2C_MEMADD_SIZE_8BIT,
                          val, 1, TAS_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef tas_select(uint8_t addr, uint8_t book, uint8_t page) {
  HAL_StatusTypeDef st = tas_write(addr, TAS_REG_PAGE, 0x00);
  st |= tas_write(addr, TAS_REG_BOOK, book);
  st |= tas_write(addr, TAS_REG_PAGE, page);
  return st;
}

HAL_StatusTypeDef tas_deep_sleep(uint8_t addr) {
  HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
  st |= tas_write(addr, TAS_REG_CTRL_2, TAS_STATE_HIZ);
  st |=
      tas_write(addr, TAS_REG_CTRL_2, TAS_STATE_DEEP_SLEEP | TAS_STATE_DIS_DSP);
  return st;
}

HAL_StatusTypeDef tas_play(uint8_t addr) {
  HAL_StatusTypeDef st;
  st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
  st |= tas_write(addr, TAS_REG_CTRL_2, TAS_STATE_HIZ);
  st |= tas_write(addr, TAS_REG_CTRL_2, TAS_STATE_PLAY);
  return st;
}

HAL_StatusTypeDef tas_set_volume(uint8_t addr, uint8_t code) {
  // -0.5 dB steps, 0x30 = 0 dB, 0xFF = mute, 0x00 = +24 dB
  HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
  st |= tas_write(addr, TAS_REG_DIG_VOL, code);
  return st;
}

/* Everything that must happen while the I2S clock is stopped. */
static HAL_StatusTypeDef tas_prepare(uint8_t addr) {
  HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);

  st |= tas_write(addr, TAS_REG_CTRL_1, TAS_STATE_384KHZ | TAS_STATE_1SPW);

  st |= tas_write(addr, TAS_REG_CTRL_2, TAS_STATE_HIZ | TAS_STATE_DIS_DSP);

  // this sets the input routing to the DSPs.
  // The TAS5805M has two DSPs, one for each channel, and each DSP has two
  // inputs. The default routing is that the left input goes to the left DSP and
  // the right input goes to the right DSP. We want both DSPs to receive the
  // same input, so we set both DSPs to receive the left input. This is done by
  // writing 0x12 to the SAP_CTRL_3 register for the left TAS5805M and 0x21 for
  // the right TAS5805M. However, this loses the ability to mix left and right
  // signals as only one input (doubled) reaches the dsp. Therefore, we use the
  // input mixer instead to achieve the same goal while retaining the ability to
  // mix left and right signals. switch (addr) {
  //   case LEFT_TAS_ADDR:  st |= tas_write(addr, TAS_REG_SAP_CTRL_3,
  //   TAS_STATE_BOTH_LEFT); break; case RIGHT_TAS_ADDR: st |= tas_write(addr,
  //   TAS_REG_SAP_CTRL_3, TAS_STATE_BOTH_RIGHT); break;
  // }

  st |= tas_write(addr, TAS_REG_AGAIN,
                  0x07); // -3.5 dB analog gain -> ~19.7 V peak output which
                         // matches the 20V supply

  st |= tas_write(addr, TAS_REG_DAC_CTRL,
                  0x78); // set DAC Frequency to 6.144 MHz instead of 3.072 MHz,
                         // because we can.

  uint8_t phase = (addr == LEFT_TAS_ADDR)
                      ? TAS_PHASE(0)
                      : TAS_PHASE(1); // 0x00 = phase 0, 0x04 = phase 1
  st |= tas_write(addr, TAS_REG_PHASE_CTRL,
                  phase | TAS_STATE_I2S_SYNC_EN | TAS_STATE_PHASE_SYNC_EN);

  st |= tas_write(addr, TAS_REG_PIN_CONTROL2, TAS_LATCHES_OFF);
  st |= tas_write(addr, TAS_REG_MISC_CONTROL, TAS_OTSD_AUTO_REC);

  // this sets the DSPs to use separate coefficients for left and right channels
  // because our "left" and "right" channels are actually the tweeter and
  // midrange channels, which are both on the same side of the soundbar.
  // Therefore, we need to use separate coefficients for each channel.
  st |= tas_write(addr, TAS_REG_DSP_MISC, TAS_STATE_LR_SEPERATE);

  return st;
}

void tas_input_mixer(uint8_t addr, uint32_t LtoL, uint32_t RtoL, uint32_t LtoR,
                     uint32_t RtoR) {
  // 9.23 fixed precision format, 0x80000000 = -256.0, 0x00800000 = 1.0,
  // 0x00000000 = 0.0
  HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_DSP, TAS_PAGE_INPUT_MIXER);
  st |= tas_write_32(addr, TAS_STATE_L_TO_L, LtoL);
  st |= tas_write_32(addr, TAS_STATE_R_TO_L, RtoL);
  st |= tas_write_32(addr, TAS_STATE_L_TO_R, LtoR);
  st |= tas_write_32(addr, TAS_STATE_R_TO_R, RtoR);

  if (st != HAL_OK)
    Error_Handler();
}

uint32_t bq_tweeter[] = {
    // Order: b0, b1, b2, -a1, -a2 (on earlevel.com a = b, b = a)
    // LR4, 3000Hz, High-Pass, Q=0.707, 0dB gain
    0x06F66F74, 0xF2132118, 0x06F66F74, 0x0DCA4A1D, 0xF9F08C4D,
    0x06F66F74, 0xF2132118, 0x06F66F74, 0x0DCA4A1D, 0xF9F08C4D,
};

uint32_t bq_midrange[] = {
    // Order: b0, b1, b2, -a1, -a2 (on earlevel.com a = b, b = a)
    // LR4, 3000Hz, Low-Pass, Q=0.707, 0dB gain
    0x00114A66, 0x002294CB, 0x00114A66, 0x0DCA4A1D, 0xF9F08C4D,
    0x00114A66, 0x002294CB, 0x00114A66, 0x0DCA4A1D, 0xF9F08C4D,
    // LR4, 100Hz, High-Pass, Q=0.707, 0dB gain
    0x07F68B2D, 0xF012E9A6, 0x07F68B2D, 0x0FED0B2D, 0xF812DE78,
    0x07F68B2D, 0xF012E9A6, 0x07F68B2D, 0x0FED0B2D, 0xF812DE78,
    // Bass lift, 300Hz, Low-Shelf, Q=0.707, +6dB gain
    0x080BC33F, 0xF03940BA, 0x07BC89CB, 0x0FC7227A, 0xF838162B,
};
#define TAS_BOOK_BQ 0xAA
#define TAS_PAGE_BQ_MIDRANGE 0x24
#define TAS_START_ADDR_BQ_MIDRANGE 0x18
#define TAS_PAGE_BQ_TWEETER 0x26
#define TAS_START_ADDR_BQ_TWEETER 0x54

void tas_equalizer(uint8_t addr, uint8_t page, uint8_t start_addr,
                   uint32_t *data, size_t length) {
  HAL_StatusTypeDef st = HAL_OK;

  st |= tas_select(addr, TAS_BOOK_BQ, page);

  uint8_t curr_addr = start_addr;
  for (int i = 0; i < length; i++) {
    st |= tas_write_32(addr, curr_addr, data[i]);
    curr_addr += sizeof(data[0]);
  }

  if (st != HAL_OK)
    Error_Handler();
}

static void tas_startup(void) {
  HAL_StatusTypeDef st = HAL_OK;
  tas_pdn(false);
  HAL_Delay(6);
  tas_pdn(true);
  HAL_Delay(6);
  stop_adc();
  HAL_Delay(6);
  st |= tas_prepare(LEFT_TAS_ADDR);
  st |= tas_prepare(RIGHT_TAS_ADDR);
  start_adc();
  HAL_Delay(20); /* oscillator + PLL     */

  /* release the DSPs (Hi-Z, DIS_DSP = 0), then wait [DS 7.5.3.1 step 4/5] */
  st |= tas_select(LEFT_TAS_ADDR, TAS_BOOK_CTRL, 0x00);
  st |= tas_select(RIGHT_TAS_ADDR, TAS_BOOK_CTRL, 0x00);
  st |= tas_write(LEFT_TAS_ADDR, TAS_REG_CTRL_2, TAS_STATE_HIZ);
  st |= tas_write(RIGHT_TAS_ADDR, TAS_REG_CTRL_2, TAS_STATE_HIZ);
  HAL_Delay(6);

  /* (DSP coefficients here) */
  // R = tweerter, L = mids -> forward left input to both channels
  tas_input_mixer(LEFT_TAS_ADDR, 0x00800000, 0x00000000, 0x00800000,
                  0x00000000);
  // R = tweerter, L = mids -> forward right input to both channels
  tas_input_mixer(RIGHT_TAS_ADDR, 0x00000000, 0x00800000, 0x00000000,
                  0x00800000);

  size_t length_midrange = sizeof(bq_midrange) / sizeof(bq_midrange[0]);
  size_t length_tweeter = sizeof(bq_tweeter) / sizeof(bq_tweeter[0]);
  tas_equalizer(LEFT_TAS_ADDR, TAS_PAGE_BQ_MIDRANGE, TAS_START_ADDR_BQ_MIDRANGE,
                bq_midrange, length_midrange);
  tas_equalizer(LEFT_TAS_ADDR, TAS_PAGE_BQ_TWEETER, TAS_START_ADDR_BQ_TWEETER,
                bq_tweeter, length_tweeter);
  tas_equalizer(RIGHT_TAS_ADDR, TAS_PAGE_BQ_MIDRANGE,
                TAS_START_ADDR_BQ_MIDRANGE, bq_midrange, length_midrange);
  tas_equalizer(RIGHT_TAS_ADDR, TAS_PAGE_BQ_TWEETER, TAS_START_ADDR_BQ_TWEETER,
                bq_tweeter, length_tweeter);

  st |= tas_select(LEFT_TAS_ADDR, TAS_BOOK_CTRL, 0x00);
  st |= tas_select(RIGHT_TAS_ADDR, TAS_BOOK_CTRL, 0x00);
  st |= tas_write(LEFT_TAS_ADDR, TAS_REG_CTRL_2, TAS_STATE_PLAY);
  st |= tas_write(RIGHT_TAS_ADDR, TAS_REG_CTRL_2, TAS_STATE_PLAY);
  HAL_Delay(6);

  /* the clock stop left a latched clock fault: clear it */
  st |= tas_write(LEFT_TAS_ADDR, TAS_REG_FAULT_CLEAR, TAS_FAULT_CLEAR);
  st |= tas_write(RIGHT_TAS_ADDR, TAS_REG_FAULT_CLEAR, TAS_FAULT_CLEAR);

  if (st != HAL_OK)
    Error_Handler();
}

void tas_handle_error(uint8_t addr, uint8_t reg, uint8_t val) {
  char buf[64];
  snprintf(buf, sizeof buf,
           "TAS5805M FAULT: addr=0x%02X, reg=0x%02X, val=0x%02X\n", addr, reg,
           val);
  reply(buf);
  // while (1) {
  //   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  //   HAL_Delay(100);
  // }
}

void tas_handle_warning(uint8_t addr, uint8_t reg, uint8_t val) {
  char buf[64];
  snprintf(buf, sizeof buf,
           "TAS5805M WARNING: addr=0x%02X, reg=0x%02X, val=0x%02X\n", addr, reg,
           val);
  reply(buf);
  // while (1) {
  //   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  //   HAL_Delay(1000);
  // }
}

void tas_faults(uint8_t addr) {
  static uint8_t chan_fault_count[2]; /* [0] left, [1] right */
  uint8_t idx = (addr == LEFT_TAS_ADDR) ? 0 : 1;
  if (chan_fault_count[idx] > TAS_MAX_CHAN_FAULTS)
    return; /* gave up, stays in Hi-Z */

  HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
  uint8_t chan = 0, g1 = 0, g2 = 0, ot = 0;
  st |= tas_read(addr, CHAN_FAULT, &chan);
  st |= tas_read(addr, GLOBAL_FAULT1, &g1);
  st |= tas_read(addr, GLOBAL_FAULT2, &g2);
  st |= tas_read(addr, OT_WARNING, &ot);
  if (st != HAL_OK)
    Error_Handler();

  if (chan)
    tas_handle_error(addr, CHAN_FAULT, chan);
  if (g1)
    tas_handle_error(addr, GLOBAL_FAULT1, g1);
  if (g2)
    tas_handle_error(addr, GLOBAL_FAULT2, g2);
  if (ot)
    tas_handle_warning(addr, OT_WARNING, ot);
  if (!(chan | g1 | g2 | ot))
    return;

  /* overcurrent / DC on a speaker output: a real hardware problem.
     Retry a few times, then give up and keep the outputs off.     */
  if (chan && ++chan_fault_count[idx] > TAS_MAX_CHAN_FAULTS) {
    tas_write(addr, TAS_REG_CTRL_2, TAS_STATE_HIZ);
    tas_handle_error(addr, CHAN_FAULT, chan); /* last report, flag kept */
    return;
  }

  /* clock, PVDD under/overvoltage, temperature: the chip recovers by
     itself [DS 7.5.3.3]; only the latched flag has to be cleared.
     If the cause is still there, the bit comes back immediately.  */
  tas_write(addr, TAS_REG_FAULT_CLEAR, TAS_FAULT_CLEAR);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2S1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // In 24/32-bit mode the Size argument counts 32-bit samples, not half-words

  tas_startup();

  pc_control_start();

  // audio_start_aux();

  // uint32_t last_poll = HAL_GetTick();
  // tas_faults_t fl, fr;

  check_aux(); // must happen after tas_startup() because it uses the I2C bus to
               // talk to the TAS5805M chips
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    if (swLeftPending &&
        (HAL_GetTick() - swLeftLastEdge) > 30) // 30 ms with no further edges
    {
      swLeftPending = 0;
      check_aux();
    }
    // if (HAL_GetTick() - last_poll >= 100) {
    //     last_poll = HAL_GetTick();
    //     if (soundbar_dsp_poll(&fl, &fr)) {
    //         /* log fl/fr over UART here, e.g. printf("%02X %02X\n", ...) */
    //         HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    //     }
    // }

    pc_control_poll();
    // static uint32_t last = 0;
    // if (HAL_GetTick() - last >= 100) {
    //     last = HAL_GetTick();
    //     tas_faults(LEFT_TAS_ADDR);
    //     tas_faults(RIGHT_TAS_ADDR);
    // }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B17DB5;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief I2S1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S1_Init(void) {

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PDN__GPIO_Port, PDN__Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_MASTER_GPIO_Port, ADC_MASTER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_EN_Pin */
  GPIO_InitStruct.Pin = BT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_PRESENT_Pin */
  GPIO_InitStruct.Pin = BT_PRESENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_PRESENT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDN__Pin */
  GPIO_InitStruct.Pin = PDN__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PDN__GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_LEFT_Pin */
  GPIO_InitStruct.Pin = SW_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_LEFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_MASTER_Pin */
  GPIO_InitStruct.Pin = ADC_MASTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_MASTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN4_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN3_Pin BTN2_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN3_Pin | BTN2_Pin | BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == SW_LEFT_Pin) {
    swLeftLastEdge = HAL_GetTick();
    swLeftPending = 1;
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == SW_LEFT_Pin) {
    swLeftLastEdge = HAL_GetTick();
    swLeftPending = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  pc_control_rx_callback(huart);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(100);
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */