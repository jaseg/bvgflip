CUBE_PATH 		?= $(wildcard ~)/resource/STM32CubeF1
CMSIS_PATH 		?= $(CUBE_PATH)/Drivers/CMSIS
CMSIS_DEV_PATH  ?= $(CMSIS_PATH)/Device/ST/STM32F1xx
HAL_PATH   		?= $(CUBE_PATH)/Drivers/STM32F1xx_HAL_Driver

CC      := arm-none-eabi-gcc
LD      := arm-none-eabi-ld
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

CFLAGS  = -g -Wall -std=gnu11 -O0 -fdump-rtl-expand
CFLAGS += -mlittle-endian -mcpu=cortex-m3 -mthumb
#CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS = -nostartfiles
#LDFLAGS += -specs=rdimon.specs -DSEMIHOSTING
LDFLAGS += -Wl,-Map=main.map -nostdlib
#LDFLAGS += -Wl,--gc-sections 
LIBS = -lgcc
#LIBS += -lrdimon

CFLAGS += -DSTM32F103xB -DHSE_VALUE=8000000

LDFLAGS += -Tstm32_flash.ld
CFLAGS += -I$(CMSIS_DEV_PATH)/Include -I$(CMSIS_PATH)/Include -I$(HAL_PATH)/Inc -Iconfig
#LDFLAGS += -L$(CMSIS_PATH)/Lib/GCC -larm_cortexM0l_math

###################################################

.PHONY: program clean

all: main.elf main.pdf

cmsis_exports.c: $(CMSIS_DEV_PATH)/Include/stm32f103xb.h $(CMSIS_PATH)/Include/core_cm3.h
	python3 gen_cmsis_exports.py $^ > $@

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $^
	$(CC) -E $(CFLAGS) -o $(@:.o=.pp) $^

%.o: %.s
	$(CC) -c $(CFLAGS) -o $@ $^
	$(CC) -E $(CFLAGS) -o $(@:.o=.pp) $^

%.dot: %.elf
	r2 -a arm -qc 'aa;agC' $< 2>/dev/null >$@

main.elf: main.o startup_stm32f103xb.o system_stm32f1xx.o $(HAL_PATH)/Src/stm32f1xx_ll_utils.o cmsis_exports.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	$(OBJDUMP) -St $@ >$(@:.elf=.lst)
	$(SIZE) $@
	
program: main.elf openocd.cfg
	openocd -f openocd.cfg -c "program $< verify reset exit"

clean:
	rm -f **.o
	rm -f main.elf main.hex main.bin main.map main.lst
	rm -f **.expand

