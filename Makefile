# CANlib Master Makefile
# Builds CANlib for various MCU configurations

# Cross-compiler toolchain.
# RepRapFirmware exports CROSS_COMPILE when building this as a submodule.
# When building CANlib standalone, fall back to a toolchain on PATH.
ARM_GNU_TOOLCHAIN_VERSION ?= 15.2.rel1
ifeq ($(OS),Windows_NT)
HOST_ARCH_RAW := $(subst AMD64,x86_64,$(subst ARM64,aarch64,$(PROCESSOR_ARCHITECTURE)))
else
HOST_ARCH_RAW := $(shell uname -m)
HOST_OS_RAW := $(shell uname -s)
endif

ifeq ($(HOST_ARCH_RAW),aarch64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),arm64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),x86_64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else ifeq ($(HOST_ARCH_RAW),amd64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else
ARM_GNU_TOOLCHAIN_HOST_ARCH := $(HOST_ARCH_RAW)
endif

ifeq ($(OS),Windows_NT)
ARM_GNU_TOOLCHAIN_HOST := mingw-w64-$(ARM_GNU_TOOLCHAIN_HOST_ARCH)
else ifeq ($(HOST_OS_RAW),Darwin)
ARM_GNU_TOOLCHAIN_HOST := darwin-$(subst aarch64,arm64,$(ARM_GNU_TOOLCHAIN_HOST_ARCH))
else
ARM_GNU_TOOLCHAIN_HOST := $(ARM_GNU_TOOLCHAIN_HOST_ARCH)
endif

CROSS_COMPILE ?= $(abspath ../arm-gnu-toolchain-$(ARM_GNU_TOOLCHAIN_VERSION)-$(ARM_GNU_TOOLCHAIN_HOST)-arm-none-eabi/bin/arm-none-eabi-)
export CROSS_COMPILE

# Toolchain commands
CXX = $(CROSS_COMPILE)g++
AR = $(CROSS_COMPILE)ar

# Quiet build support (Linux kernel style)
# Use V=1 for verbose output
ifeq ($(V),1)
	Q :=
else
	Q := @
endif
export Q

# Recursive wildcard: $(call rwildcard,<dir>,<patterns>)
# Source lists must not shell out to find, which resolves to FIND.EXE on Windows
rwildcard = $(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

# An empty scan would archive no objects and then look up to date forever, so stop here instead
ifeq ($(wildcard src/*),)
$(error No sources found under src - is the checkout complete?)
endif

# Available build configurations
CONFIGS := SAME70_RTOS SAME70_nonRTOS SAME51_RTOS SAME51_nonRTOS SAMC21_nonRTOS SAMC21_RTOS SAM4E_RTOS RP2040_RTOS STM32H5_RTOS STM32H7_RTOS

# Default target
.DEFAULT_GOAL := SAME70_RTOS

# Print available targets
.PHONY: help
help:
	@echo "CANlib Build System"
	@echo "Available targets:"
	@for config in $(CONFIGS); do echo "  make $$config"; done
	@echo ""
	@echo "Other targets:"
	@echo "  make all          - Build all configurations"
	@echo "  make clean        - Clean all build outputs"
	@echo "  make clean-<config> - Clean specific configuration"
	@echo ""
	@echo "Options:"
	@echo "  V=1               - Verbose build output"

# Build all configurations
.PHONY: all
all:
	$(Q)$(MAKE) SAME70_RTOS
	$(Q)$(MAKE) SAME51_RTOS
	$(Q)$(MAKE) SAM4E_RTOS
	$(Q)$(MAKE) SAMC21_RTOS

# Include configuration-specific makefiles only when building that specific config
ifeq ($(MAKECMDGOALS),SAME70_RTOS)
-include Makefiles/SAME70_RTOS.mk
endif
ifeq ($(MAKECMDGOALS),SAME70_nonRTOS)
-include Makefiles/SAME70_nonRTOS.mk
endif
ifeq ($(MAKECMDGOALS),SAME51_RTOS)
-include Makefiles/SAME51_RTOS.mk
endif
ifeq ($(MAKECMDGOALS),SAME51_nonRTOS)
-include Makefiles/SAME51_nonRTOS.mk
endif
ifeq ($(MAKECMDGOALS),SAMC21_nonRTOS)
-include Makefiles/SAMC21_nonRTOS.mk
endif
ifeq ($(MAKECMDGOALS),SAMC21_RTOS)
-include Makefiles/SAMC21_RTOS.mk
endif
ifeq ($(MAKECMDGOALS),SAM4E_RTOS)
-include Makefiles/SAM4E_RTOS.mk
endif
ifeq ($(MAKECMDGOALS),RP2040_RTOS)
-include Makefiles/RP2040_RTOS.mk
endif
ifeq ($(MAKECMDGOALS),STM32H5_RTOS)
-include Makefiles/STM32H5_RTOS.mk
endif
ifeq ($(MAKECMDGOALS),STM32H7_RTOS)
-include Makefiles/STM32H7_RTOS.mk
endif

# Generic clean target
.PHONY: clean
clean:
	@echo "Cleaning all CANlib build outputs..."
	@for config in $(CONFIGS); do \
		if [ -d "$$config" ]; then \
			echo "  Cleaning $$config..."; \
			rm -rf "$$config"; \
		fi; \
	done
