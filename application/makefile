#
# Makafile used to build free rtos demo
# Author: Felipe Silva Neves
#
TOOLCHAN_PATH=/Users/felipeneves/Documents/toolchains/arm-gcc/bin/
PREFIX=arm-none-eabi-
CC=gcc
LD=g++
AS=as
GDB=gdb
CFLAGS= -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv5-sp-d16  -g -c -O0
SFLAGS= -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv5-sp-d16  -g -O0
OUTFILE= flow_sensor
SIZ = size
SIZEFLAGS = --format=berkeley

#
# Add include directories:
#
CFLAGS += -I ../os/Source/include
CFLAGS += -I ../os/Source/portable/GCC/ARM_CM4F
CFLAGS += -I proj_include
CFLAGS += -I proj_include/cmsis
CFLAGS += -I drivers
CFLAGS += -I debug_util
CFLAGS += -I board
CFLAGS += -I ov7670
CFLAGS += -I ../usb_cdc -I ../usb_cdc/usb_stack/device
CFLAGS += -I ../usb_cdc/usb_stack/osa -DUSB_STACK_FREERTOS
CFLAGS += -I ../usb_cdc/usb_stack/include





#
# Add source directories:
#


# the os sources
SRC  = $(wildcard ../os/Source/*.c)
SRC += $(wildcard ../os/Source/portable/GCC/ARM_CM4F/*.c)
SRC += ../os/Source/portable/MemMang/heap_2.c

# add drivers folder:
SRC += $(wildcard drivers/*.c)

# add usb stack:
SRC += $(wildcard ../usb_cdc/*.c)
SRC += $(wildcard ../usb_cdc/usb_stack/device/*.c)
SRC += ../usb_cdc/usb_stack/osa/usb_osa_freertos.c


#add debug drivers:
SRC += $(wildcard debug_util/*.c)

#add bsp files:
SRC += $(wildcard board/*.c)

#add main or other c file stuff
SRC += $(wildcard *.c)
SRC += $(wildcard system/*.c)

#add custom camera source files
SRC += $(wildcard ov7670/*.c)

#add startup code:
AS_SRC = $(wildcard system/*.s)

#
# Define linker script files:
#
LDS=system/linker_script.ld
LDFLAGS =  -g -mthumb -mcpu=cortex-m4  -mfloat-abi=hard -mfpu=fpv5-sp-d16 --specs=nosys.specs  -Wl,-static,--gc-sections
LDFLAGS += -T$(LDS) -Xlinker -Map=$(OUTFILE).map
LIBS = -lm


#
# .c to .o recursion magic:
#
OBJS  = $(SRC:.c=.o)
OBJS += $(AS_SRC:.S=.o)


#
# Define the build chain:
#
.PHONY: all, clean,connect,debug_jlink

all: $(OUTFILE).elf
	@echo "[ELF]: Generated $< file successfully!"

connect:

debug_jlink: $(OUTFILE).elf
	@echo "starting debugger!"



clean:
	@echo "[CLEAN]: Cleaning !"
	@rm -f *.elf
	@rm -f  ../os/Source/*.o
	@rm -f  ../os/Source/portable/GCC/ARM_CM4F/*.o
	@rm -f  ../os/Source/portable/MemMang/*.o
	@rm -f  ../usb_cdc/*.o
	@rm -f ../usb_cdc/usb_stack/device/*.o
	@rm -f ../usb_cdc/usb_stack/osa/*.o
	@rm -f  system/*.o
	@rm -f  *.o
	@rm -f  drivers/*.o
	@rm -f  debug_util/*.o
	@rm -f  board/*.o
	@rm -f  ov7670/*.o
	@rm -f  *.map
	@echo "[CLEAN]: Done !"



#
# Linking step:
#
$(OUTFILE).elf: $(OBJS)
	@echo "[LD]: Linking files!"
	@$(TOOLCHAN_PATH)$(PREFIX)$(LD) $(LDFLAGS) -Xlinker --start-group $(OBJS) $(LIBS) -Xlinker --end-group -o $@
	@echo "[LD]: Cleaning intermediate files!"
	@rm -f  ../os/Source/*.o
	@rm -f  ../os/Source/portable/GCC/ARM_CM4F/*.o
	@rm -f  ../os/Source/portable/MemMang/*.o
	@rm -f  ../usb_cdc/*.o
	@rm -f ../usb_cdc/usb_stack/device/*.o
	@rm -f ../usb_cdc/usb_stack/osa/*.o
	@rm -f  system/*.o
	@rm -f  *.o
	@rm -f  drivers/*.o
	@rm -f  debug_util/*.o
	@rm -f  board/*.o
	@rm -f  ov7670/*.o


#
# Compiling step:
#
.c.o:
	@echo "[CC]: $< "
	@$(TOOLCHAN_PATH)$(PREFIX)$(CC) $(CFLAGS) -o $@  $<

#
# startup code step:
#
.s.o:
	@echo "[AS]: $< "
	@$(TOOLCHAN_PATH)$(PREFIX)$(CC) $(SFLAGS) -o $@  $<
