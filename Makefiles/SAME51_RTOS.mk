# CANlib SAME51_RTOS Configuration Makefile

BUILD_DIR := SAME51_RTOS
TARGET := $(BUILD_DIR)/libCANlib.a

SRC_DIR := src

CPP_SRCS := $(shell find $(SRC_DIR) -name '*.cpp')

INCLUDES := \
	-I$(SRC_DIR) \
	-I../RRFLibraries/src \
	-I../CoreN2G/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM7/r0p1

DEFINES := \
	-D__SAME51N19A__ \
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
	-Werror=return-type \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-fstack-usage \
	-fdump-rtl-expand \
	-O2 \
	-Wall \
	$(INCLUDES) \
	$(DEFINES)

OBJS := $(CPP_SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

.PHONY: SAME51_RTOS
SAME51_RTOS: $(TARGET)

$(TARGET): $(OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MP -o $@ $<

-include $(DEPS)

.PHONY: clean-SAME51_RTOS
clean-SAME51_RTOS:
	$(Q)echo "  RM      $(BUILD_DIR)"
	$(Q)rm -rf $(BUILD_DIR)
