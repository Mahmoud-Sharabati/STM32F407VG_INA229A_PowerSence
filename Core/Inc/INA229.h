/*
 * INA229.h
 *
 *  Created on: Sep 11, 2022
 *      Author: Mahmoud
 */

#ifndef INC_INA229_H_
#define INC_INA229_H_

#include "main.h"

#define INA229_config_register 0x00U
#define INA229_config_register_rst_NormalOperation 0x0000U
#define INA229_config_register_rst_SystemReset 0x8000U
#define INA229_config_register_rstacc_NormalOperation 0x0000U
#define INA229_config_register_rstacc_ClearENERGYandCHARGEregisters 0x4000U
#define INA229_config_register_tempcomp_Shunttemperaturecompensationdisabled 0x0000U
#define INA229_config_register_tempcomp_Shunttemperaturecompensationenabled 0x0020U
#define INA229_config_register_adcrange_16384mV 0x0000U
#define INA229_config_register_adcrange_4096mV 0x0010U
#define INA229_adc_config_register 0x01U
#define INA229_adc_config_register_mode_Shutdown0 0x0000U
#define INA229_adc_config_register_mode_Triggeredbusvoltagesingleshot 0x1000U
#define INA229_adc_config_register_mode_Triggeredshuntvoltagesingleshot 0x2000U
#define INA229_adc_config_register_mode_Triggeredshuntvoltageandbusvoltagesingleshot 0x3000U
#define INA229_adc_config_register_mode_Triggeredtemperaturesingleshot 0x4000U
#define INA229_adc_config_register_mode_Triggeredtemperatureandbusvoltagesingleshot 0x5000U
#define INA229_adc_config_register_mode_Triggeredtemperatureandshuntvoltagesingleshot 0x6000U
#define INA229_adc_config_register_mode_Triggeredbusvoltageshuntvoltageandtemperaturesingleshot 0x7000U
#define INA229_adc_config_register_mode_Shutdown1 0x8000U
#define INA229_adc_config_register_mode_Continuousbusvoltageonly 0x9000U
#define INA229_adc_config_register_mode_Continuousshuntvoltageonly 0xA000U
#define INA229_adc_config_register_mode_Continuousshuntandbusvoltage 0xB000U
#define INA229_adc_config_register_mode_Continuoustemperatureonly 0xC000U
#define INA229_adc_config_register_mode_Continuousbusvoltageandtemperature 0xD000U
#define INA229_adc_config_register_mode_Continuoustemperatureandshuntvoltage 0xE000U
#define INA229_adc_config_register_mode_Continuousbusvoltageshuntvoltageandtemperature 0xF000U
#define INA229_adc_config_register_vbusct_50us 0x0000U
#define INA229_adc_config_register_vbusct_84us 0x0200U
#define INA229_adc_config_register_vbusct_150us 0x0400U
#define INA229_adc_config_register_vbusct_280us 0x0600U
#define INA229_adc_config_register_vbusct_540us 0x0800U
#define INA229_adc_config_register_vbusct_1052us 0x0A00U
#define INA229_adc_config_register_vbusct_2074us 0x0C00U
#define INA229_adc_config_register_vbusct_4120us 0x0E00U
#define INA229_adc_config_register_vshct_50us 0x0000U
#define INA229_adc_config_register_vshct_84us 0x0040U
#define INA229_adc_config_register_vshct_150us 0x0080U
#define INA229_adc_config_register_vshct_280us 0x00C0U
#define INA229_adc_config_register_vshct_540us 0x0100U
#define INA229_adc_config_register_vshct_1052us 0x0140U
#define INA229_adc_config_register_vshct_2074us 0x0180U
#define INA229_adc_config_register_vshct_4120us 0x01C0U
#define INA229_adc_config_register_vtct_50us 0x0000U
#define INA229_adc_config_register_vtct_84us 0x0008U
#define INA229_adc_config_register_vtct_150us 0x0010U
#define INA229_adc_config_register_vtct_280us 0x0018U
#define INA229_adc_config_register_vtct_540us 0x0020U
#define INA229_adc_config_register_vtct_1052us 0x0028U
#define INA229_adc_config_register_vtct_2074us 0x0030U
#define INA229_adc_config_register_vtct_4120us 0x0038U
#define INA229_adc_config_register_avg_1 0x0000U
#define INA229_adc_config_register_avg_4 0x0001U
#define INA229_adc_config_register_avg_16 0x0002U
#define INA229_adc_config_register_avg_64 0x0003U
#define INA229_adc_config_register_avg_128 0x0004U
#define INA229_adc_config_register_avg_256 0x0005U
#define INA229_adc_config_register_avg_512 0x0006U
#define INA229_adc_config_register_avg_1024 0x0007U
#define INA229_shunt_cal_register 0x02U
#define INA229_shunt_cal_register_reserved0_ENABLE 0x8000U
#define INA229_shunt_cal_register_reserved0_DISABLE 0x0000U
#define INA229_shunt_tempco_register 0x03U
#define INA229_vshunt_register 0x04U
#define INA229_vbus_register 0x05U
#define INA229_dietemp_register 0x06U
#define INA229_current_register 0x07U
#define INA229_power_register 0x08U
#define INA229_energy_register 0x09U
#define INA229_charge_register 0x0AU
#define INA229_diag_alrt_register 0x0BU
#define INA229_diag_alrt_register_alatch_Transparent 0x0000U
#define INA229_diag_alrt_register_alatch_LatchedAlertpin 0x8000U
#define INA229_diag_alrt_register_cnvr_DisableconversionreadyflagonALERTpin 0x0000U
#define INA229_diag_alrt_register_cnvr_EnablesconversionreadyflagonALERTpin 0x4000U
#define INA229_diag_alrt_register_slowalert_ALERTcomparisononnonaveragedADCvalue 0x0000U
#define INA229_diag_alrt_register_slowalert_ALERTcomparisononaveragedvalue 0x2000U
#define INA229_diag_alrt_register_apol_Normalactivelowopendrain 0x0000U
#define INA229_diag_alrt_register_apol_Invertedactivehighopendrain 0x1000U
#define INA229_diag_alrt_register_energyof_ENABLE 0x0800U
#define INA229_diag_alrt_register_energyof_DISABLE 0x0000U
#define INA229_diag_alrt_register_chargeof_ENABLE 0x0400U
#define INA229_diag_alrt_register_chargeof_DISABLE 0x0000U
#define INA229_diag_alrt_register_mathof_ENABLE 0x0200U
#define INA229_diag_alrt_register_mathof_DISABLE 0x0000U
#define INA229_diag_alrt_register_reserved0_ENABLE 0x0100U
#define INA229_diag_alrt_register_reserved0_DISABLE 0x0000U
#define INA229_diag_alrt_register_tmpol_ENABLE 0x0080U
#define INA229_diag_alrt_register_tmpol_DISABLE 0x0000U
#define INA229_diag_alrt_register_shntol_ENABLE 0x0040U
#define INA229_diag_alrt_register_shntol_DISABLE 0x0000U
#define INA229_diag_alrt_register_shntul_ENABLE 0x0020U
#define INA229_diag_alrt_register_shntul_DISABLE 0x0000U
#define INA229_diag_alrt_register_busol_ENABLE 0x0010U
#define INA229_diag_alrt_register_busol_DISABLE 0x0000U
#define INA229_diag_alrt_register_busul_ENABLE 0x0008U
#define INA229_diag_alrt_register_busul_DISABLE 0x0000U
#define INA229_diag_alrt_register_pol_ENABLE 0x0004U
#define INA229_diag_alrt_register_pol_DISABLE 0x0000U
#define INA229_diag_alrt_register_cnvrf_ENABLE 0x0002U
#define INA229_diag_alrt_register_cnvrf_DISABLE 0x0000U
#define INA229_diag_alrt_register_memstat_ENABLE 0x0001U
#define INA229_diag_alrt_register_memstat_DISABLE 0x0000U
#define INA229_sovl_register 0x0CU
#define INA229_suvl_register 0x0DU
#define INA229_bovl_register 0x0EU
#define INA229_bovl_register_reserved0_ENABLE 0x8000U
#define INA229_bovl_register_reserved0_DISABLE 0x0000U
#define INA229_buvl_register 0x0FU
#define INA229_buvl_register_reserved0_ENABLE 0x8000U
#define INA229_buvl_register_reserved0_DISABLE 0x0000U
#define INA229_temp_limit_register 0x10U
#define INA229_pwr_limit_register 0x11U
#define INA229_manufacturer_id_register 0x3EU
#define INA229_device_id_register 0x3FU

#define INA229_CS_HIGH()    	(HAL_GPIO_WritePin(INA229_SPI_CS_GPIO_Port, INA229_SPI_CS_Pin, GPIO_PIN_SET))
#define INA229_CS_LOW()     	(HAL_GPIO_WritePin(INA229_SPI_CS_GPIO_Port, INA229_SPI_CS_Pin, GPIO_PIN_RESET))

typedef struct INA299_Readings {
	/* INA299 variables */
	float VSHUNT_mV;
	float VBUS_V;
	float CURRENT_A;
	float DIETEMP_C;

} INA299_Readings;

/* Private variables ---------------------------------------------------------*/
extern  void Error_Handler(void);

/*
 *  ======== INA229_State ========
 *  Initial configuration state for a INA229 sensor
 */
typedef struct INA229_State {
	uint16_t configRegister;
	uint16_t adcConfigRegister;
	uint16_t shuntCalRegister;
	uint16_t shuntTempcoRegister;
	uint16_t diagAlrtRegister;
	uint16_t sovlRegister;
	uint16_t suvlRegister;
	uint16_t bovlRegister;
	uint16_t buvlRegister;
	uint16_t tempLimitRegister;
	uint16_t pwrLimitRegister;

	uint16_t adcrange; //config_register_adcrange
	float currentlsb; //current lsb value

	uint8_t busId;   /* SPI bus id */
	uint8_t devCS;   /* Sensor's SPI chip select id */

	uint16_t osWait; /* One shot conversion time (in ms)  */
} INA229_State;

/*
 *  ======== INA229_Handle ========
 *  First argument to all INA229 methods
 */
typedef INA229_State *INA229_Handle;

/*
 *  ======== INA229_writeReg ========
 * Write register
 */
extern void INA229_writeReg(INA229_Handle sensor, uint8_t regAddr, uint16_t value);

/*
 *  ======== INA229_config ========
 *  Configure device with current settings
 */
extern void INA229_config(INA229_Handle sensor);

/*
 *  ======== INA229_setCURRENT_LSB ========
 *  Set the CURRENT_LSB value used for calculations
 */
extern void INA229_setCURRENT_LSB(INA229_Handle sensor, float CURRENT_LSB);

/*
 *  ======== INA229_readReg ========
 *  Read register
 */
extern uint64_t INA229_readReg(INA229_Handle sensor, uint8_t regAddr);

/*
 *  ======== INA229_getVSHUNT_mV ========
 *  Get VSHUNT value (mV)
 */
extern float INA229_getVSHUNT_mV(INA229_Handle sensor);

/*
 *  ======== INA229_getVBUS_V ========
 *  Get VBUS value (V)
 */
extern float INA229_getVBUS_V(INA229_Handle sensor);

/*
 *  ======== INA229_getDIETEMP_C ========
 *  Get DIETMEP value (C)
 */
extern float INA229_getDIETEMP_C(INA229_Handle sensor);

/*
 *  ======== INA229_getDIETEMP_F ========
 *  Get DIETMEP value (F)
 */
extern float INA229_getDIETEMP_F(INA229_Handle sensor);

/*
 *  ======== INA229_getCURRENT_signedLSB ========
 *  Get CURRENT value (signed value in LSBs)
 */
extern float INA229_getCURRENT_signedLSB(INA229_Handle sensor);

/*
 *  ======== INA229_getCURRENT_A ========
 *  Get CURRENT value (A)
 */
extern float INA229_getCURRENT_A(INA229_Handle sensor);

/*
 *  ======== INA229_getPOWER_signedLSB ========
 *  Get POWER value (signed value in LSBs)
 */
extern float INA229_getPOWER_signedLSB(INA229_Handle sensor);

/*
 *  ======== INA229_getPOWER_W ========
 *  Get POWER value (W)
 */
extern float INA229_getPOWER_W(INA229_Handle sensor);

/*
 *  ======== INA229_getENERGY_signedLSB ========
 *  Get ENERGY value (signed value in LSBs)
 */
extern double INA229_getENERGY_signedLSB(INA229_Handle sensor);

/*
 *  ======== INA229_getENERGY_J ========
 *  Get ENERGY value (J)
 */
extern double INA229_getENERGY_J(INA229_Handle sensor);

/*
 *  ======== INA229_getCHARGE_signedLSB ========
 *  Get CHARGE value (signed value in LSBs)
 */
extern double INA229_getCHARGE_signedLSB(INA229_Handle sensor);

/*
 *  ======== INA229_getCHARGE_C ========
 *  Get CHARGE value (C)
 */
extern double INA229_getCHARGE_C(INA229_Handle sensor);

/*---------------------------_MCU Functions_--------------------------------*/
/*
 *  ======== mcu_spiInit ========
 *  Initialize the specified SPI bus for first use
 */
void INA229_Init(SPI_HandleTypeDef * hspi, TIM_HandleTypeDef *htim);
/*
 * ======== mcu_spiTransfer ========
 * Transfer data to and from a SPI slave
 *
 * @param busId id of an SPI bus to access for the transfer
 * @param count number of frames for this transaction
 * @param txBuf buffer with the data to be transmitted
 * @param rxBuf buffer to receive data
 *
 * @return      0 if successful, otherwise non-zero
 */

extern void mcu_spiTransfer(uint8_t busId, uint8_t csGPIOId, uint8_t count, uint8_t *txBuf);

/*
 *  ======== mcu_msWait ========
 *  Delay CPU for at least the specified number of milliseconds
 *
 *  @param msWait - number of milliseconds to delay, a value of 0 causes
 *                  this function to return immediately.
 */
extern void mcu_msWait(unsigned long msWait);

extern INA299_Readings Get_INA299_Readings(void);

#endif /* INC_INA229_H_ */
