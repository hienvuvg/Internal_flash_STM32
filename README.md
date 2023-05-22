# Internal flash of STM32
Worked with LoRa WLxx SoCs, expected to work with low-power L4xx MCUs.

In F410, need to write into the final sector of the flash memory (previous sectors are occupied by the program). When programming, all sectors related to the program will be erased. Thus, need to ensure the user sector are not used by the program.
