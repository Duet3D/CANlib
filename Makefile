# CANlib Master Makefile
# Builds CANlib for various MCU configurations

# Cross-compiler toolchain (relative to project root)
#CROSS_COMPILE ?= ../arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
CROSS_COMPILE ?= ../arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
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

# Available build configurations
CONFIGS := SAME70_RTOS SAME51_RTOS SAME51_nonRTOS SAMC21_nonRTOS SAMC21_RTOS SAM4E_RTOS RP2040_RTOS

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
