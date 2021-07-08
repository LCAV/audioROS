# List of the ChibiOS generic KL2x startup and CMSIS files.
STARTUPSRC = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/crt1.c \
             $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/vectors.c

STARTUPASM = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/crt0_v6m.s

STARTUPINC = $(CHIBIOS)/os/common/ports/ARMCMx/devices/KL2x \
             $(CHIBIOS)/os/ext/CMSIS/include \
             $(CHIBIOS)/os/ext/CMSIS/KINETIS

STARTUPLD  = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/ld
