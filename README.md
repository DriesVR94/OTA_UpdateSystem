# OTA_UpdateSystem
Here we are making an over-the-air (OTA) update system for the Aether CubeSat of KU Leuven!

## Project Description
In this project, we simulate installing a software update on a CubeSat. We are using two NUCLEO boards: One board represents the On-Board Computer (OBC), the other board represents the telemetry module of the CubeSat. (Typically, the telemetry module takes care of communication between the ground station and the CubeSat.)
This project focuses on the implementation of the update system. Hence, the OTA communication part is out of the scope of this project. Imagine an OTA update has been sent from the ground station to the CubeSat. The telemetry module will send this update package through a CAN bus to the CubeSat's OBC. Here, the necessary steps will be taken to install, verify, and run the update.

For a detailed description of the project, we refer to our thesis "Design and Implementation of a Robust and Flexible Over-The-Air (OTA) Update System for a 3U CubeSat." 
DISCLAIMER: Due to continuous development, there can be small inconsistencies between the thesis and the code in this repo.

## Recommended Software
- STM32CubeIDE (for development)
- STM32CubeProgrammer (for erasing and/or programming flash memory)

## Hardware Used in this Project
- 2 x NUCLEO-F446ZE boards
- 2 x MCP2551 CAN transceivers
- 1 x W25QXX external flash memory

This is just a list of the hardware that we used. Feel free to use other stuff where possible!

## User's Manual
1. Start by erasing the flash memory of both NUCLEO boards. This can easily be done with the STM32CubeProgrammer.
2. Pick 1 of the NUCLEO boards (doesn't matter which one) and program a .bin file containing a software update into flash sector 6 of the board. Other sectors are possible too, but you have to change the code then!
3. Program the 'Telemetry' project in the same board by running its main.c file.
4. Take the second board and program the 'FreeRTOS' project by running its main.c file. It's important to program the 'FreeRTOS' project first!
5. Also in the second board, now program the 'Bootloader' project by running its main.c file.
6. Connect both boards to a serial terminal such as Putty to follow what is going on. At this point, you should be able to see that the applications of the FreeRTOS are running.
7. When you press the blue user's button of the first board, the data transmission will start. After a while, the update will be installed. At that point, the system will reboot and the update starts running.
   

If you have any questions regarding our project, feel free to contact us!
