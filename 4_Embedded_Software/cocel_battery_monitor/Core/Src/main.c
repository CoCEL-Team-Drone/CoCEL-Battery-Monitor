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
/**
  ******************************************************************************
		Implemented by Sanghyun Park, Eunseon Choi @ CoCEL, POSTECH, South Korea
		(pash0302@postech.ac.kr, eunseon103@postech.ac.kr)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
============================================================
					  Memory Address
============================================================
*/
#define BQ76942_ADDR  0x10

#define DM_VCELL_MODE       0x9304 // Vcell mode

// CONFIG_UPDATE mode
#define BATT_STATUS_REG     0x12
#define BATT_CFGUPDATE      0x0001

// Protection
#define DM_ENABLED_PROT_A      0x9261
#define DM_ENABLED_PROT_B      0x9262
#define DM_ENABLED_PROT_C      0x9263

// Cell balancing
#define DM_BALANCING_CONFIGURATION   0x9335

// Enable ARM_SW
#define DM_DFETOFF_PIN_CONFIG    0x92FB

// Enable TS1, TS3 thermistor
#define DM_TS1_CONFIG    0x92FD
#define DM_TS3_CONFIG    0x92FFU

#define UART_TX_BUF_SIZE 128U
#define UART_RX_BUF_SIZE 64

/*
============================================================
						Command
============================================================
*/
// CONFIG_UPDATE mode
#define CMD_SET_CFGUPDATE   0x0090
#define CMD_EXIT_CFGUPDATE  0x0092

// FET control
#define CMD_FET_ENABLE            0x0022
#define CMD_MANUFACTURING_STATUS  0x0057
#define CMD_ALL_FETS_ON           0x0096
#define MFG_STATUS_FET_EN_MASK    0x0010

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct
{
	uint32_t timestamp;
	uint32_t read_out_time;
	uint16_t cell1;
	uint16_t cell2;
	uint16_t cell3;
	uint16_t cell4;
	uint16_t cell5;
	uint16_t cell6;

	uint16_t stack_voltage_raw;

	int16_t  current_raw;
	int16_t  ext_temp_raw;
} BattRawData;

// for connect timer interrupt <-> while loop
volatile uint8_t TI_SENSOR_RECORD_FLAG = 0;
volatile uint8_t TI_UART_TRANSMIT_FLAG = 0;

volatile BattRawData batt_data;

volatile uint16_t cell1, cell2, cell3, cell4, cell5, cell6;
volatile uint16_t stack_voltage_raw;
volatile int16_t current_raw;
volatile int16_t int_temp_raw;

volatile HAL_StatusTypeDef i2c_status;
volatile uint32_t i2c_error;
volatile HAL_StatusTypeDef bq_ready;


/*
============================================================
				    For UART Interface - START
============================================================
*/
// for system initialization
volatile uint8_t IS_INITIALIZED = 0;
volatile uint8_t CELL_NUM = 4;
volatile uint8_t SET_HZ = 10;
volatile uint8_t TI_HZ_MAX_CNT = 0;
volatile uint8_t TEMP_UNIT = 1; // K, C
// for system operation
volatile uint8_t EX_RECORD_FLAG = 0;
volatile uint8_t ARM_STATUS = 0;

// transmit data format
typedef struct
{
	uint32_t timestamp;
	uint32_t read_out_time;
	int16_t status;
	uint16_t data_1;
	uint16_t data_2;
	uint16_t data_3;
	uint16_t data_4;
	uint16_t data_5;
	uint16_t data_6;
	uint16_t data_7;
	int16_t data_etc_1;
	int16_t data_etc_2;
}UartTransmitData;

// receive data format
typedef struct
{
	int32_t type;
	int32_t data_1;
	int32_t data_2;
	int32_t data_3;
}UartReceiveData;

volatile UartTransmitData data_transmit;
volatile UartReceiveData data_receive;

UartTransmitData data_transmit_for_warning;

// for receive
volatile uint8_t data_receive_ready = 0;
volatile uint8_t uart_parse_error = 0;
static uint8_t uart_rx_byte;
static char uart_build_buffer[UART_RX_BUF_SIZE];
static uint16_t uart_build_index = 0;
static char uart_line_buffer[UART_RX_BUF_SIZE];
static volatile uint8_t uart_line_ready = 0;

/*
============================================================
				    For UART Interface - END
============================================================
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// Direct command read functions
HAL_StatusTypeDef BQ_ReadReg(uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef BQ_ReadU8(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef BQ_ReadU16(uint8_t reg, uint16_t *value);
HAL_StatusTypeDef BQ_ReadI16(uint8_t reg, int16_t *value);

// Data Memory read/write functions
HAL_StatusTypeDef BQ_WriteDataMemoryU8(uint16_t addr, uint8_t value);
HAL_StatusTypeDef BQ_ReadDataMemoryU8(uint16_t addr, uint8_t *value);
HAL_StatusTypeDef BQ_WriteDataMemoryU16(uint16_t addr, uint16_t value);
HAL_StatusTypeDef BQ_ReadDataMemoryU16(uint16_t addr, uint16_t *value);

// BQ76942 configuration functions
HAL_StatusTypeDef BQ_SetVcellMode(uint8_t cell_count);
HAL_StatusTypeDef BQ_DisableCellBalancing(void);
HAL_StatusTypeDef BQ_DisableProtections(void);
HAL_StatusTypeDef BQ_EnableFETs(void);
HAL_StatusTypeDef BQ_SetDFETOFFPin(void);
HAL_StatusTypeDef BQ_SetTS1Thermistor(void);
HAL_StatusTypeDef BQ_SetTS3Thermistor(void);

// Subcommand / command helper functions
HAL_StatusTypeDef BQ_ReadSubcommandU16(uint16_t subcmd, uint16_t *value);
HAL_StatusTypeDef BQ_WriteSubcommand(uint16_t subcmd);
uint8_t BQ_Checksum(uint16_t addr, uint8_t *data, uint8_t len);
uint8_t BQ_WaitCfgUpdate(uint8_t enter);
HAL_StatusTypeDef BQ_WaitCfgUpdate(uint8_t enter);

// get data
HAL_StatusTypeDef BQ_ReadBattRawData(BattRawData *data);

// Debug / UART helper functions
void UART_Print(char *msg);
void UART_TransmitParsedData(const UartTransmitData *tx_data);
void clear_tx_data(UartTransmitData *tx_data);
HAL_StatusTypeDef UART_GetReceiveData(UartReceiveData *output);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 * Direct Command Read Functions
 * ============================================================ */
// common read
HAL_StatusTypeDef BQ_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U)) return HAL_ERROR;

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        BQ76942_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100
    );

    i2c_status = status;
    i2c_error = HAL_I2C_GetError(&hi2c1);

    return status;
}
// read unsigned 8bit
HAL_StatusTypeDef BQ_ReadU8(uint8_t reg, uint8_t *value)
{
    if (value == NULL)return HAL_ERROR;
    return BQ_ReadReg(reg, value, 1);
}
// read unsigned 16bit
HAL_StatusTypeDef BQ_ReadU16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    if (value == NULL)return HAL_ERROR;

    status = BQ_ReadReg(reg, data, 2);

    if (status != HAL_OK)return status;

    *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    return HAL_OK;
}
// read signed 16bit
HAL_StatusTypeDef BQ_ReadI16(uint8_t reg, int16_t *value)
{
    uint8_t data[2];
    uint16_t raw;
    HAL_StatusTypeDef status;

    if (value == NULL)return HAL_ERROR;

    status = BQ_ReadReg(reg, data, 2);

    if (status != HAL_OK)return status;

    raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    *value = (int16_t)raw;

    return HAL_OK;
}

/* ============================================================
 * Data Memory Read / Write Functions
 * ============================================================ */
HAL_StatusTypeDef BQ_WriteDataMemoryU8(uint16_t addr, uint8_t value)
{
    uint8_t addr_buf[2];
    uint8_t data_buf[1];
    uint8_t chk_len[2];

    HAL_StatusTypeDef status;

    addr_buf[0] = (uint8_t)(addr & 0xFFU);
    addr_buf[1] = (uint8_t)(addr >> 8);

    data_buf[0] = value;

    chk_len[0] = BQ_Checksum(addr, data_buf, 1);
    chk_len[1] = 5U;

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x3E,
        I2C_MEMADD_SIZE_8BIT,
        addr_buf,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(2);

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x40,
        I2C_MEMADD_SIZE_8BIT,
        data_buf,
        1,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(2);


    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x60,
        I2C_MEMADD_SIZE_8BIT,
        chk_len,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(10);

    return HAL_OK;
}

HAL_StatusTypeDef BQ_ReadDataMemoryU8(uint16_t addr, uint8_t *value)
{
    uint8_t addr_buf[2];

    HAL_StatusTypeDef status;

    if (value == NULL) return HAL_ERROR;

    addr_buf[0] = (uint8_t)(addr & 0xFFU);
    addr_buf[1] = (uint8_t)(addr >> 8);

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x3E,
        I2C_MEMADD_SIZE_8BIT,
        addr_buf,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(10);

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        BQ76942_ADDR,
        0x40,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        100
    );

    return status;
}

HAL_StatusTypeDef BQ_WriteDataMemoryU16(uint16_t addr, uint16_t value)
{
    uint8_t addr_buf[2];
    uint8_t data_buf[2];
    uint8_t chk_len[2];

    HAL_StatusTypeDef status;

    addr_buf[0] = (uint8_t)(addr & 0xFFU);
    addr_buf[1] = (uint8_t)(addr >> 8);

    data_buf[0] = (uint8_t)(value & 0xFFU);
    data_buf[1] = (uint8_t)(value >> 8);

    chk_len[0] = BQ_Checksum(addr, data_buf, 2);
    chk_len[1] = 6U;

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x3E,
        I2C_MEMADD_SIZE_8BIT,
        addr_buf,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(2);

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x40,
        I2C_MEMADD_SIZE_8BIT,
        data_buf,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(2);

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x60,
        I2C_MEMADD_SIZE_8BIT,
        chk_len,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(10);

    return HAL_OK;
}

HAL_StatusTypeDef BQ_ReadDataMemoryU16(uint16_t addr, uint16_t *value)
{
    uint8_t addr_buf[2];
    uint8_t data_buf[2];

    HAL_StatusTypeDef status;

    if (value == NULL) return HAL_ERROR;

    addr_buf[0] = (uint8_t)(addr & 0xFFU);
    addr_buf[1] = (uint8_t)(addr >> 8);

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x3E,
        I2C_MEMADD_SIZE_8BIT,
        addr_buf,
        2,
        100
    );

    if (status != HAL_OK) return status;
    HAL_Delay(10);

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        BQ76942_ADDR,
        0x40,
        I2C_MEMADD_SIZE_8BIT,
        data_buf,
        2,
        100
    );

    if (status != HAL_OK) return status;

    *value =
        (uint16_t)data_buf[0] |
        ((uint16_t)data_buf[1] << 8);

    return HAL_OK;
}
/* ============================================================
 * BQ76942 Configuration Functions
 * ============================================================ */
HAL_StatusTypeDef BQ_DisableProtections(void)
{
    uint8_t pa = 0U;
    uint8_t pb = 0U;
    uint8_t pc = 0U;

    char buf[120];

    if (BQ_WriteSubcommand(CMD_SET_CFGUPDATE) != HAL_OK)
    {
//    	UART_Print("BQ_WriteSubcommand\r\n");
    	return HAL_ERROR;
    }

    if (BQ_WaitCfgUpdate(1) != HAL_OK)
    {
//        UART_Print("CFGUPDATE enter failed\r\n");
        return HAL_ERROR;
    }


    if (BQ_WriteDataMemoryU8(
            DM_ENABLED_PROT_A,
            0x00) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_WriteDataMemoryU8(
            DM_ENABLED_PROT_B,
            0x00) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_WriteDataMemoryU8(
            DM_ENABLED_PROT_C,
            0x00) != HAL_OK)
    {
        return HAL_ERROR;
    }


    if (BQ_ReadDataMemoryU8(
            DM_ENABLED_PROT_A,
            &pa) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_ReadDataMemoryU8(
            DM_ENABLED_PROT_B,
            &pb) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_ReadDataMemoryU8(
            DM_ENABLED_PROT_C,
            &pc) != HAL_OK)
    {
        return HAL_ERROR;
    }

//    snprintf(
//        buf,
//        sizeof(buf),
//        "Protection RB PA=0x%02X PB=0x%02X PC=0x%02X\r\n",
//        pa,
//        pb,
//        pc
//    );
//
//    UART_Print(buf);

    BQ_WriteSubcommand(CMD_EXIT_CFGUPDATE);

    if (BQ_WaitCfgUpdate(0U) != HAL_OK)
    {
//        UART_Print("CFGUPDATE exit failed\r\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Vcell Mode setting
HAL_StatusTypeDef BQ_SetVcellMode(uint8_t cell_count)
{
	uint16_t mode;
	uint16_t rb = 0U;
	char buf[120];

	if (cell_count == 4U)
	{
		mode = 0x000F; // Cell1~Cell4 enabled
	}
	else if (cell_count == 6U)
	{
		mode = 0x030F; // Cell1~4,Cell9~10 // mode = 0x003F; // Cell1~Cell6 enabled
	}
	else
	{
		return HAL_ERROR;
	}


	if (BQ_WriteSubcommand(CMD_SET_CFGUPDATE) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (BQ_WaitCfgUpdate(1U) != HAL_OK)
	{
//		UART_Print("CFGUPDATE enter failed\r\n");
		return HAL_ERROR;
	}


	if (BQ_WriteDataMemoryU16(
			DM_VCELL_MODE,
			mode) != HAL_OK)
	{
		return HAL_ERROR;
	}


	if (BQ_ReadDataMemoryU16(
			DM_VCELL_MODE,
			&rb) != HAL_OK)
	{
		return HAL_ERROR;
	}


	if (rb != mode)
	{
//		UART_Print("VcellMode verify failed\r\n");
		return HAL_ERROR;
	}


//	snprintf(
//		buf,
//		sizeof(buf),
//		"VcellMode set=0x%04X readback=0x%04X\r\n",
//		mode,
//		rb
//	);
//
//	UART_Print(buf);


	if (BQ_WriteSubcommand(CMD_EXIT_CFGUPDATE) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (BQ_WaitCfgUpdate(0U) != HAL_OK)
	{
//		UART_Print("CFGUPDATE exit failed\r\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

// disable cell balanceing
HAL_StatusTypeDef BQ_DisableCellBalancing(void)
{
    uint8_t rb = 0U;

    if(BQ_WriteSubcommand(CMD_SET_CFGUPDATE)!=HAL_OK)
    {
//    	UART_Print("BQ_DisableCellBalancing-0 Error\r\n");
        return HAL_ERROR;
    }

    if (BQ_WaitCfgUpdate(1)!= HAL_OK)
    {
//    	UART_Print("BQ_DisableCellBalancing-1 Error\r\n");
        return HAL_ERROR;
    }


    if (BQ_WriteDataMemoryU8(
            DM_BALANCING_CONFIGURATION,
            0x10) != HAL_OK)
    {
//    	UART_Print("BQ_DisableCellBalancing-2 Error\r\n");
        return HAL_ERROR;
    }


    if (BQ_ReadDataMemoryU8(
            DM_BALANCING_CONFIGURATION,
            &rb) != HAL_OK)
    {
//    	UART_Print("BQ_DisableCellBalancing-3 Error\r\n");
        return HAL_ERROR;
    }


    if (rb != 0x10U)
    {
//    	UART_Print("BQ_DisableCellBalancing-4 Error\r\n");
        return HAL_ERROR;
    }


    BQ_WriteSubcommand(CMD_EXIT_CFGUPDATE);

    if (BQ_WaitCfgUpdate(0)!= HAL_OK)
    {
//    	UART_Print("BQ_DisableCellBalancing-5 Error\r\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Enable DFETOFF pin(ARM SW)
HAL_StatusTypeDef BQ_SetDFETOFFPin(void)
{
    uint8_t rb = 0U;

    /* Enter CONFIG_UPDATE */
    if (BQ_WriteSubcommand(CMD_SET_CFGUPDATE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_WaitCfgUpdate(1U) != HAL_OK)
    {
        return HAL_ERROR;
    }


    /* DFETOFF function, active-low */
    if (BQ_WriteDataMemoryU8(
            DM_DFETOFF_PIN_CONFIG,
            0x82U) != HAL_OK)
    {
        return HAL_ERROR;
    }


    /* Readback */
    if (BQ_ReadDataMemoryU8(
            DM_DFETOFF_PIN_CONFIG,
            &rb) != HAL_OK)
    {
        return HAL_ERROR;
    }
    else
    {
//        char bufd[64];
//
//        sprintf(bufd,
//                 "DFETOFF Config = 0x%02X\r\n",
//                 rb);

//        UART_Print(bufd);
    }

    if (rb != 0x82U)
    {
        return HAL_ERROR;
    }


    /* Exit CONFIG_UPDATE */
    if (BQ_WriteSubcommand(CMD_EXIT_CFGUPDATE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_WaitCfgUpdate(0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Enable FET
HAL_StatusTypeDef BQ_EnableFETs(void)
{
    uint16_t mfg_status = 0U;

    if (BQ_ReadSubcommandU16(CMD_MANUFACTURING_STATUS, &mfg_status) != HAL_OK) return HAL_ERROR;

    if ((mfg_status & MFG_STATUS_FET_EN_MASK) == 0U)
    {
    	if(BQ_WriteSubcommand(CMD_FET_ENABLE) != HAL_OK) return HAL_ERROR;
        HAL_Delay(2);
        if (BQ_ReadSubcommandU16(CMD_MANUFACTURING_STATUS, &mfg_status) != HAL_OK) return HAL_ERROR;
        if ((mfg_status & MFG_STATUS_FET_EN_MASK) == 0U) return HAL_ERROR;

    }

    if(BQ_WriteSubcommand(CMD_ALL_FETS_ON) != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);

    return HAL_OK;
}

// Enable externer thermistor
HAL_StatusTypeDef BQ_SetTS1Thermistor(void)
{
    uint8_t rb = 0U;
    HAL_StatusTypeDef status;


    /* Enter CONFIG_UPDATE */
    status = BQ_WriteSubcommand(CMD_SET_CFGUPDATE);

    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ_WaitCfgUpdate(1U);

    if (status != HAL_OK)
    {
        return status;
    }


    /*
     * TS1 Config = 0x0B
     *
     * Internal pull-up : 18 kOhm
     * Temperature model: 18K model
     * Measurement type : Temperature report only
     * Pin function      : Thermistor
     */
    status = BQ_WriteDataMemoryU8(
        DM_TS1_CONFIG,
        0x0BU
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* Readback */
    status = BQ_ReadDataMemoryU8(
        DM_TS1_CONFIG,
        &rb
    );

    if (status != HAL_OK)
    {
        return status;
    }

    if (rb != 0x0BU)
    {
        return HAL_ERROR;
    }


    /* Exit CONFIG_UPDATE */
    status = BQ_WriteSubcommand(CMD_EXIT_CFGUPDATE);

    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ_WaitCfgUpdate(0U);

    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}
HAL_StatusTypeDef BQ_SetTS3Thermistor(void)
{
    uint8_t rb = 0U;

    if (BQ_WriteSubcommand(CMD_SET_CFGUPDATE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_WaitCfgUpdate(1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 10k NTC, internal 18k pull-up, temperature report */
    if (BQ_WriteDataMemoryU8(
            DM_TS3_CONFIG,
            0x0BU) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_ReadDataMemoryU8(
            DM_TS3_CONFIG,
            &rb) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (rb != 0x0BU)
    {
        return HAL_ERROR;
    }

    if (BQ_WriteSubcommand(CMD_EXIT_CFGUPDATE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BQ_WaitCfgUpdate(0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
/* ============================================================
 * Subcommand / Command Helper Functions
 * ============================================================ */
HAL_StatusTypeDef BQ_ReadSubcommandU16(uint16_t subcmd, uint16_t *value)
{
    uint8_t cmd[2];
    uint8_t data[2];

    if (value == NULL) return HAL_ERROR;

    cmd[0] = (uint8_t)(subcmd & 0xFFU);
    cmd[1] = (uint8_t)((subcmd >> 8) & 0xFFU);

    if (HAL_I2C_Mem_Write(
            &hi2c1,
            BQ76942_ADDR,
            0x3E,
            I2C_MEMADD_SIZE_8BIT,
            cmd,
            2,
            100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(2);

    if (HAL_I2C_Mem_Read(
            &hi2c1,
            BQ76942_ADDR,
            0x40,
            I2C_MEMADD_SIZE_8BIT,
            data,
            2,
            100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *value =
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8);

    return HAL_OK;
}

HAL_StatusTypeDef BQ_WriteSubcommand(uint16_t subcmd)
{
    uint8_t data[2];

    data[0] = (uint8_t)(subcmd & 0xFFU);
    data[1] = (uint8_t)((subcmd >> 8) & 0xFFU);

    return HAL_I2C_Mem_Write(
        &hi2c1,
        BQ76942_ADDR,
        0x3E,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        100
    );
}

uint8_t BQ_Checksum(uint16_t addr, uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;

    sum += (addr & 0xFF);
    sum += (addr >> 8);

    for(uint8_t i=0;i<len;i++)
        sum += data[i];

    return (uint8_t)(0xFF - (sum & 0xFF));
}

// CONFIG_UPDATE Mode - Tech. Ref. Manu. 7.6
HAL_StatusTypeDef BQ_WaitCfgUpdate(uint8_t enter)
{
    uint16_t batt = 0U;

    for (uint8_t i = 0U; i < 50U; i++)
    {
        if (BQ_ReadU16(BATT_STATUS_REG, &batt) != HAL_OK)
        {
//        	UART_Print("BQ_ReadU16 Failed \r\n");
            return HAL_ERROR;
        }

        if (enter != 0U)
        {
            /* CONFIG_UPDATE 진입 확인 */
            if ((batt & BATT_CFGUPDATE) != 0U) return HAL_OK;
        }
        else
        {
            /* CONFIG_UPDATE 종료 확인 */
            if ((batt & BATT_CFGUPDATE) == 0U) return HAL_OK;
        }

        HAL_Delay(20);
    }
//    UART_Print("BQ_WaitCfgUpdate Timeout \r\n");
    return HAL_TIMEOUT;
}

/* ============================================================
 * Debug / UART Helper Functions
 * ============================================================ */
HAL_StatusTypeDef BQ_ReadBattRawData(BattRawData *data)
{
	HAL_StatusTypeDef status;

	if (data == NULL) return HAL_ERROR;

	data->timestamp = HAL_GetTick();

	/*
	 * cell 1: 0x14 => vc1 - vc0
	 * cell 2: 0x16 => vc2 - vc1
	 * cell 3: 0x18 => vc3 - vc2
	 * cell 4: 0x1A => vc4 - vc3
	 * cell 5: 0x1C => vc5 - vc4
	 * cell 6: 0x1E => vc6 - vc5
	 * cell 7: 0x20 => vc7 - vc6
	 * cell 8: 0x22 => vc8 - vc7
	 * cell 9: 0x24 => vc9 - vc8
	 * cell 10: 0x26 => vc10 - vc9
	 */

		status = BQ_ReadU16(0x14, &data->cell1); // cell 1
		if (status != HAL_OK) return status;

		status = BQ_ReadU16(0x16, &data->cell2); // cell 2
		if (status != HAL_OK) return status;

		if(CELL_NUM == 6)
		{
			status = BQ_ReadU16(0x18, &data->cell3); // cell 3
			if (status != HAL_OK) return status;

			status = BQ_ReadU16(0x1A, &data->cell4); // cell 4
			if (status != HAL_OK) return status;

			status = BQ_ReadU16(0x1C, &data->cell5); // cell 5
			if (status != HAL_OK) return status;

			status = BQ_ReadU16(0x26, &data->cell6); // cell 10
			if (status != HAL_OK) return status;
		}
		else
		{
			status = BQ_ReadU16(0x18, &data->cell3); // cell 3
			if (status != HAL_OK) return status;

			status = BQ_ReadU16(0x26, &data->cell4); // cell 10
			if (status != HAL_OK) return status;

			data->cell5 = 0;
			data->cell6 = 0;
		}

	status = BQ_ReadU16(0x34, &data->stack_voltage_raw);
	if (status != HAL_OK) return status;

	status = BQ_ReadI16(0x3A, &data->current_raw);
	if (status != HAL_OK) return status;

	status = BQ_ReadI16(0x70, &data->ext_temp_raw);
	if (status != HAL_OK) return status;

	uint32_t tmp_time = HAL_GetTick();
	data->read_out_time = tmp_time - data->timestamp;

	return HAL_OK;
}

void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
void UART_TransmitParsedData(const UartTransmitData *tx_data)
{
	char buf[UART_TX_BUF_SIZE];
	int len;

	if (tx_data == NULL)return;
	len = snprintf(
	        buf,
	        sizeof(buf),
	        "%lu,%lu,%d,%u,%u,%u,%u,%u,%u,%u,%d,%d\r\n",

	        (unsigned long)tx_data->timestamp,
					(unsigned long)tx_data->read_out_time,

	        tx_data->status,

	        tx_data->data_1,
	        tx_data->data_2,
	        tx_data->data_3,
	        tx_data->data_4,
	        tx_data->data_5,
	        tx_data->data_6,
					tx_data->data_7,

	        tx_data->data_etc_1,
	        tx_data->data_etc_2
	    );

	if ((len > 0) && (len < (int)sizeof(buf)))
	{
			HAL_UART_Transmit(
					&huart1,
					(uint8_t *)buf,
					(uint16_t)len,
					100
			);
	}
}
void clear_tx_data(UartTransmitData *tx_data)
{
	tx_data->timestamp = 0;
	tx_data->read_out_time = 0;
	tx_data->status = 1;
	tx_data->data_1 = 0;
	tx_data->data_2 = 0;
	tx_data->data_3 = 0;
	tx_data->data_4 = 0;
	tx_data->data_5 = 0;
	tx_data->data_6 = 0;
	tx_data->data_7 = 0;
	tx_data->data_etc_1 = 0;
	tx_data->data_etc_2 = 0;
}

HAL_StatusTypeDef UART_GetReceiveData(UartReceiveData *output)
{
    char local_buffer[UART_RX_BUF_SIZE];
    UartReceiveData parsed_data;
    char extra_character;
    int parse_count;

    if (output == NULL)return HAL_ERROR;

    if (uart_line_ready == 0)return HAL_BUSY;

    memcpy(local_buffer,
           uart_line_buffer,
           sizeof(local_buffer));

    uart_line_ready = 0;

    parse_count = sscanf(
        local_buffer,
        " %" SCNd32
        " %" SCNd32
        " %" SCNd32
        " %" SCNd32
        " %c",
        &parsed_data.type,
        &parsed_data.data_1,
        &parsed_data.data_2,
        &parsed_data.data_3,
        &extra_character
    );

    if (parse_count != 4)return HAL_ERROR;

    *output = parsed_data;

    return HAL_OK;
}

// 10Hz
volatile uint32_t ti_hz_cnt = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		HAL_StatusTypeDef status;
		status = UART_GetReceiveData(&data_receive);
		if (status == HAL_OK)
		{
			// received data = initialization signal
			if(data_receive.type == 1)
			{
				// if initialized => already initialized, but received data is about initialization
				clear_tx_data(&data_transmit_for_warning);
				data_transmit_for_warning.timestamp = HAL_GetTick();
				data_transmit_for_warning.status = 1;
				if(IS_INITIALIZED == 1)
				{
					data_transmit_for_warning.data_1 = 3;
					data_transmit_for_warning.data_2 = 2;
					data_transmit_for_warning.data_3 = 1;
					TI_UART_TRANSMIT_FLAG = 1;
					return;
				}

				// if not initialized
				int8_t need_retry = 0;
				int32_t user_cell_num = data_receive.data_1;
				int32_t user_logging_hz = data_receive.data_2;
				int32_t user_ther_unit = data_receive.data_3;
				data_transmit_for_warning.data_1 = 3;
				data_transmit_for_warning.data_2 = 2;
				data_transmit_for_warning.data_3 = 2;
					// validate data
				if(user_cell_num != 4 && user_cell_num != 6)
				{
					need_retry = 1;
					data_transmit_for_warning.data_4 = 1;
				}
				if(user_logging_hz > 10 || user_logging_hz <= 0)
				{
					need_retry = 1;
					data_transmit_for_warning.data_5 = 1;
				}
				if(user_ther_unit != 1 && user_ther_unit != 2)
				{
					need_retry = 1;
					data_transmit_for_warning.data_6 = 1;
				}
				if(need_retry == 1)
				{
					UART_TransmitParsedData(&data_transmit_for_warning);
					return;
				}
					// setting
				CELL_NUM = (uint8_t)user_cell_num;
				SET_HZ = (uint8_t)user_logging_hz;
				TI_HZ_MAX_CNT = 10 / SET_HZ;
				TEMP_UNIT = (uint8_t)user_ther_unit;
				IS_INITIALIZED = 1;
			}
			// need to initialize. send type=1
			if(IS_INITIALIZED == 0)
			{
				clear_tx_data(&data_transmit_for_warning);
				data_transmit_for_warning.timestamp = HAL_GetTick();
				data_transmit_for_warning.status = 1;
				data_transmit_for_warning.data_1 = 3;
				data_transmit_for_warning.data_2 = 2;
				data_transmit_for_warning.data_3 = 3;
				UART_TransmitParsedData(&data_transmit_for_warning);
				return;
			}
			// system handling
			else if(data_receive.type == 2)
			{
				clear_tx_data(&data_transmit_for_warning);
				data_transmit_for_warning.timestamp = HAL_GetTick();
				data_transmit_for_warning.status = 1;
				data_transmit_for_warning.data_1 = 2;
				data_transmit_for_warning.data_2 = 2;

				// Start / Stop Logging
				if(data_receive.data_1 == 1 && EX_RECORD_FLAG == 0)
				{
					EX_RECORD_FLAG = 1;
					data_transmit_for_warning.data_3 = 1;
					TI_UART_TRANSMIT_FLAG = 1;
				}
				else if(data_receive.data_1 == 2 && EX_RECORD_FLAG == 1)
				{
					EX_RECORD_FLAG = 0;
					data_transmit_for_warning.data_3 = 2;
					TI_UART_TRANSMIT_FLAG = 1;
				}

				// ARM SW Off/On
				if(data_receive.data_2 == 1 && ARM_STATUS == 0)
				{
					HAL_GPIO_WritePin(ARM_SW_GPIO_Port, ARM_SW_Pin, GPIO_PIN_SET);
					ARM_STATUS = 1;
					data_transmit_for_warning.data_4 = 1;
					TI_UART_TRANSMIT_FLAG = 1;
				}
				else if(data_receive.data_2 == 2 && ARM_STATUS == 1)
				{
					HAL_GPIO_WritePin(ARM_SW_GPIO_Port, ARM_SW_Pin, GPIO_PIN_RESET);
					ARM_STATUS = 0;
					data_transmit_for_warning.data_4 = 2;
					TI_UART_TRANSMIT_FLAG = 1;
				}
			}
			// request info
			else if(data_receive.type == 3)
			{
				// TODO
			}
		}

		// invalid uart data format
		else if(status == HAL_ERROR)
		{
			clear_tx_data(&data_transmit_for_warning);
			data_transmit_for_warning.timestamp = HAL_GetTick();
			data_transmit_for_warning.status = 1;
			data_transmit_for_warning.data_1 = 3;
			data_transmit_for_warning.data_2 = 1;
			data_transmit_for_warning.data_3 = 1;
			if(IS_INITIALIZED == 0)UART_TransmitParsedData(&data_transmit_for_warning);
			else if(IS_INITIALIZED == 1)TI_UART_TRANSMIT_FLAG = 1;
		}

		//
		ti_hz_cnt++;
		if(ti_hz_cnt >= TI_HZ_MAX_CNT)
		{
			ti_hz_cnt = 0;
			if(bq_ready == HAL_OK && EX_RECORD_FLAG == 1) TI_SENSOR_RECORD_FLAG = 1;
		}

	}
}

// UART Interrupt
volatile HAL_StatusTypeDef uart_start_status = (HAL_StatusTypeDef)0xFF;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		char ch = (char)uart_rx_byte;
		if (ch == '\n')
		{
				if (uart_build_index > 0)
				{
						uart_build_buffer[uart_build_index] = '\0';

						if (uart_line_ready == 0)
						{
								memcpy(uart_line_buffer,
											 uart_build_buffer,
											 uart_build_index + 1);

								uart_line_ready = 1;
						}
				}

				uart_build_index = 0;
		}
		else if (ch != '\r')
		{
				if (uart_build_index < UART_RX_BUF_SIZE - 1)
				{
						uart_build_buffer[uart_build_index++] = ch;
				}
				else
				{
						uart_build_index = 0;
				}
		}

		HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
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
  /* USER CODE BEGIN 2 */
  uart_start_status = HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  bq_ready = HAL_ERROR;
  HAL_TIM_Base_Start_IT(&htim2); // initialize timer interrupt
  HAL_GPIO_WritePin(ARM_SW_GPIO_Port, ARM_SW_Pin, GPIO_PIN_RESET);
  ARM_STATUS = 0;
  HAL_Delay(300);

  clear_tx_data(&data_transmit);
  data_transmit.status = 1;
  data_transmit.data_1 = 1; // initializating

  // wait for initialize
  while(1)
  {
  	if(IS_INITIALIZED == 1)break;

		data_transmit.timestamp = HAL_GetTick();
		data_transmit.data_2 = 1;
		data_transmit.data_3 = 1;
		UART_TransmitParsedData(&data_transmit);

  	HAL_Delay(1000);
  }

  // Initialize sensor module pre-setting
  data_transmit.timestamp = HAL_GetTick();
  data_transmit.data_2 = 2;
  if(BQ_SetVcellMode(CELL_NUM) != HAL_OK)data_transmit.data_3 = 1; //UART_Print("BQ_SerVcellMode Error\r\n");
  if(BQ_DisableCellBalancing() != HAL_OK)data_transmit.data_4 = 1; // UART_Print("BQ_DisableCellBalancing Error\r\n");
  if(BQ_DisableProtections() != HAL_OK)data_transmit.data_5 = 1; // UART_Print("BQ_DisableProtections Error\r\n");
  if(BQ_SetDFETOFFPin() != HAL_OK)data_transmit.data_6 = 1; //UART_Print("BQ_SetDFETOFFPin Error\r\n");
  if(BQ_SetTS1Thermistor() != HAL_OK)data_transmit.data_7 = 1; //UART_Print("BQ_SetTS1Thermistor Error\r\n");
  if(BQ_EnableFETs() != HAL_OK)data_transmit.data_etc_1 = 1; // UART_Print("BQ_EnableFETs Error\r\n");
  UART_TransmitParsedData(&data_transmit);
  HAL_Delay(100);

  // Initialize sensor module
  clear_tx_data(&data_transmit);
  data_transmit.timestamp = HAL_GetTick();
  data_transmit.data_1 = 1;
  bq_ready = HAL_I2C_IsDeviceReady(&hi2c1, BQ76942_ADDR, 3, 100);
  if(bq_ready != HAL_OK)
  {
  	data_transmit.status = 2;
  	data_transmit.data_2 = 1;
  	UART_TransmitParsedData(&data_transmit);
  	return -1;
  }
  data_transmit.status = 1;
  data_transmit.data_2 = 3;
  data_transmit.data_3 = (uint16_t)CELL_NUM;
  data_transmit.data_4 = (uint16_t)SET_HZ;
  data_transmit.data_5 = (uint16_t)TEMP_UNIT;

  UART_TransmitParsedData(&data_transmit);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	if(TI_UART_TRANSMIT_FLAG == 1)
  	{
  		UART_TransmitParsedData(&data_transmit_for_warning);
  		TI_UART_TRANSMIT_FLAG = 0;
  	}

	  if(TI_SENSOR_RECORD_FLAG == 0) continue;
	  TI_SENSOR_RECORD_FLAG = 0;


		//
		clear_tx_data(&data_transmit);

		//
		if(BQ_ReadBattRawData(&batt_data) != HAL_OK)
		{
//			UART_Print("BQ sensor read error\r\n");
			data_transmit.timestamp = batt_data.timestamp;
			data_transmit.status = 1;
			data_transmit.data_1 = 2;
			data_transmit.data_2 = 1;
			data_transmit.data_3 = 1;
			UART_TransmitParsedData(&data_transmit);
			continue;
		}

		uint8_t safety_status = 0U;
		uint8_t fet_status = 0U;

		uint16_t control_status = 0U;
		uint16_t battery_status = 0U;

//		if (BQ_ReadU8(0x03, &safety_status) != HAL_OK)
//		{
//		  UART_Print("Safety Status read error\r\n");
//		  continue;
//		}
//
//
//		if (BQ_ReadU16(0x00, &control_status) != HAL_OK)
//		{
//		  UART_Print("Control Status read error\r\n");
//		  continue;
//		}
//
//
//		if (BQ_ReadU16(0x12, &battery_status) != HAL_OK)
//		{
//		  UART_Print("Battery Status read error\r\n");
//		  continue;
//		}
//
//
//		if (BQ_ReadU8(0x7F, &fet_status) != HAL_OK)
//		{
//		  UART_Print("FET Status read error\r\n");
//		  continue;
//		}

		// normal data
		data_transmit.timestamp = batt_data.timestamp;
		data_transmit.read_out_time = batt_data.read_out_time;
		data_transmit.status = 0;
		data_transmit.data_1 = batt_data.cell1;
		data_transmit.data_2 = batt_data.cell2;
		data_transmit.data_3 = batt_data.cell3;
		data_transmit.data_4 = batt_data.cell4;
		data_transmit.data_5 = batt_data.cell5;
		data_transmit.data_6 = batt_data.cell6;
		data_transmit.data_7 = batt_data.stack_voltage_raw;
		data_transmit.data_etc_1 = -batt_data.current_raw;
		if(TEMP_UNIT == 2)
		{
			data_transmit.data_etc_2 = batt_data.ext_temp_raw - 2731;
		}
		else
		{
			data_transmit.data_etc_2 = batt_data.ext_temp_raw;
		}

		UART_TransmitParsedData(&data_transmit);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ARM_SW_GPIO_Port, ARM_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARM_SW_Pin */
  GPIO_InitStruct.Pin = ARM_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ARM_SW_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
