# STM32F407VG_INA229A_PowerSence
This repository contains a driver for the TI-INA229 Power/Energy/Charge Monitor, designed to interface with the STM32f407VG microcontroller

## Requirements
* A device with an STM32f407VG microcontroller.
* A device with an TI-INA229 Power/Energy/Charge Monitor.
* The STM32CubeMX software, available [here](https://www.st.com/en/development-tools/stm32cubemx.html).

## Installation
1. Clone the repository:
git clone [here](https://github.com/Mahmoud-Sharabati/STM32F407VG_INA229A_PowerSence.git).
2. Import the project into STM32CubeMX by selecting File > Import > C/C++ > Existing Code as Makefile Project.
3. Configure the project in STM32CubeMX according to your hardware setup.
4. Generate the code and open the project in your preferred development environment.
5. Build and upload the code to your device.

## Usage

### Before using SPI CAN Driver
1. Modify the Hardware Configuration and Pins using CubeMX software
    This driver use SPI2 with the following pins:
    | SPI2_CS       | SPI2_SCK    | SPI2_MISO   |SPI2_MOSI   |
    | ------------- |:-----------:|:-----------:|:----------:|
    | PE9	    | PB10	  | PC2        | PC3	     |

	#### Notes:																	
	1. You need to modify the name of "Chip-Select pin CS" on your project to be "INA229_SPI_CS".
	2. You must modify the SPI parameters as shown in image below.
	\
	![SPI_Configuration](https://user-images.githubusercontent.com/16566502/215982637-edbc0245-69df-4847-aca3-c270d2754e99.png).
	3. Other parameters will be automatically modified when generates the code.
	4. This driver uses TIMER6 for delay purpose. Any other timer can be used after its configuration is done.

2. This Board has an optional LED and Buzzer with the following pins:
   | LED	| Buzzer  |
   |------------|---------|
   | PB0	| PE2	  |

 ### Use INA229 Driver instruction
1. Define your INA299_Readings variable to be used to store the INA229 readings. The driver project uses variable nemed "INA299_Values"

2. Initialize the INA229 driver using: INA229_Init(SPI_HandleTypeDef * hspi, TIM_HandleTypeDef *htim)

	Parameters:
   | hspi | htim |
   |------------|--------- |
   | Connected SPI_HandleTypeDef | Used TIM_HandleTypeDef for delay |
   
3. Get the INA229 readings by Get_INA299_Readings() function. The diver project uses the "INA299_Values" variable to store reading values into. 

_NOTE: The INA229 readings in this project can be monitored using the STM32 debugger.

## Credits
This driver is based on the generated code by the SysConfig tool for the TI Sensors. It can be found [here](https://www.ti.com/tool/SYSCONFIG).

## License
This driver is licensed under the MIT License. See the LICENSE file for details.
