## FreeRTOS Household Energy Management Software

+ The **FreeRTOS/Source** directory contains the FreeRTOS source code.

+ The **FreeRTOS/Demo** directory contains the application.

To run it, you will need to have QEMU installed. After that, clone this repository to your machine, go to the directory Demo/CORTEX_MPS2_QEMU_IAR_GCC/build/gcc, run make, and run qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -kernel ./output/RTOSDemo.out -monitor none -nographic -serial stdio. You should see the application start.

To see the application code, go to Demo/CORTEX_MPS2_QEMU_IAR_GCC and open main.c.
