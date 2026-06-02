# CANlib SAM4E_RTOS Configuration Makefile

BUILD_DIR := SAM4E_RTOS
TARGET := $(BUILD_DIR)/libCANlib.a

SRC_DIR := src

CPP_SRCS := $(shell find $(SRC_DIR) -name '*.cpp')

INCLUDES := \
	-I$(SRC_DIR) \
	-I../RRFLibraries/src \
	-I../CoreN2G/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM4F

DEFINES := \
	-D__SAM4E8E__ \
	-DRTOS

CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-fno-threadsafe-statics \
	-fno-rtti \
	-fno-exceptions \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-Wfloat-conversion \
	-Werror=return-type \
	-Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-fstack-usage \
	-Os \
	-Wall \
	-Werror \
	-Wnoexcept \
	-Wshadow \
	-Wsign-promo \
	$(INCLUDES) \
	$(DEFINES)

OBJS := $(CPP_SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

.PHONY: SAM4E_RTOS
SAM4E_RTOS: $(TARGET)

$(TARGET): $(OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MP -o $@ $<

-include $(DEPS)

.PHONY: clean-SAM4E_RTOS
clean-SAM4E_RTOS:
	$(Q)echo "  RM      $(BUILD_DIR)"
	$(Q)rm -rf $(BUILD_DIR)
