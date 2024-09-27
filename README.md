## FreeRTOS Household Energy Management Software

+ The **FreeRTOS/Source** directory contains the FreeRTOS source code.

+ The **FreeRTOS/Demo** directory contains the application.

To run it, you will need to have QEMU installed. After that, clone this repository to your machine, go to the directory Demo/CORTEX_MPS2_QEMU_IAR_GCC/build/gcc, run make, and run qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -kernel ./output/RTOSDemo.out -monitor none -nographic -serial stdio. You should see the application start.

To see the application code, go to Demo/CORTEX_MPS2_QEMU_IAR_GCC and open main.c.


The application manages energy expenditure in a household with solar panels. 
Simulates energy being generated from solar panels. Simulates a battery that is charged with said energy, and can power devices within the household. When the battery is full, the energy is automatically sold to the grid (at an estimated price). When its capacity is not enought to fulfil the needs of the household, energy is bought from the grid.

Each second in running code is equivalent to one hour of real life time, to speed up visualisation (one day is simulated in 24 seconds of runtime).
