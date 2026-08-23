TARGET := hil_stm32_f446re
BUILD := build
CROSS ?= arm-none-eabi-
KERNEL := third_party/FreeRTOS-Kernel
COMMON := ../../common

CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
WARNINGS := -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
TEST_WATCHDOG ?= 0
BAUD ?= 115200
CFLAGS := $(CPU) -std=c11 $(WARNINGS) -ffreestanding -fdata-sections -ffunction-sections \
  -fno-common -DSTM32F446xx -D__FPU_PRESENT=1U
CFLAGS += -Isrc -Iconfig -I$(COMMON)/hil_protocol/include -I$(COMMON)/hil_control/include \
  -I$(KERNEL)/include -I$(KERNEL)/portable/GCC/ARM_CM4F
CFLAGS += -Iinclude -I$(COMMON)/hil_safety/include -DHIL_TEST_WATCHDOG=$(TEST_WATCHDOG) -DUART_BAUD_RATE=$(BAUD)
LDFLAGS := $(CPU) -Tlinker/stm32f446re.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map \
  --specs=nano.specs --specs=nosys.specs

SOURCES := \
  src/main.c \
  src/startup_stm32f446xx.c \
  src/timing.c \
  $(COMMON)/hil_protocol/src/hil_protocol.c \
  $(COMMON)/hil_control/src/hil_control.c \
  $(COMMON)/hil_safety/src/hil_safety.c \
  $(KERNEL)/tasks.c \
  $(KERNEL)/queue.c \
  $(KERNEL)/list.c \
  $(KERNEL)/stream_buffer.c \
  $(KERNEL)/portable/GCC/ARM_CM4F/port.c
OBJECTS := $(patsubst %.c,$(BUILD)/%.o,$(SOURCES))

.PHONY: all clean size
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex size

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CROSS)gcc $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) linker/stm32f446re.ld
	@mkdir -p $(dir $@)
	$(CROSS)gcc $(LDFLAGS) $(OBJECTS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(CROSS)objcopy -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(CROSS)objcopy -O ihex $< $@

size: $(BUILD)/$(TARGET).elf
	$(CROSS)size $<

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
