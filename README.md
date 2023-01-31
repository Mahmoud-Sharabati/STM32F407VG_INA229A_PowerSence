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
	1. You only need to modify the name of "Chip-Select pin CS" on your project to be "INA229_SPI_CS".
	2. Other parameters will be automatically modified when generates the code.
	3. This driver uses TIMER6 for delay purpose. Any other timer can be used after its configuration is done.

2. This Board has an optional LED and Buzzer with the following pins:
   | LED	| Buzzer  |
   |------------|---------|
   | PB0	| PE2	  |

 ### Use INA229 Driver instruction
1. Initialize the INA229 driver using: INA229_Init(SPI_HandleTypeDef * hspi, TIM_HandleTypeDef *htim)

	Parameters:
   | hspi | htim |
   |------------|--------- |
   | Connected SPI_HandleTypeDef | Used TIM_HandleTypeDef for delay |
   
2. Enable/Disable RX pins Interrupt (on RX0B and RX1B pins)

	Two available options:
   | Enable Interrupts on RX0B and RX1B pins	| Disable Interrupts on RX0B and RX1B pins  |
   |--------------------------------------------|-------------------------------------------|
   | _Enable					| _Disable	  			    |

_NOTE: When using the RX interrupts, the interrupt flag MUST be cleared by software using CANSPI_RXB0_CLR Function_
 
3. Use Transmission and Receive functions:\
Transmission Function:\
`CAN_AddTxMessage(CAN_HeaderTypeDef *tempCanMsg)`\
Receive Function:\
`CAN_GetRxMessage(CAN_HeaderTypeDef *tempCanMsg)`

## Credits
This driver is based on the work of Daniel Rossi, whose original driver can be found [here](https://github.com/ProjectoOfficial/STM32/tree/main/STM32_MCP2515).

## License
This driver is licensed under the MIT License. See the LICENSE file for details.
