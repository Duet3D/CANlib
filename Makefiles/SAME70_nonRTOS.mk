# CANlib SAME70_nonRTOS Configuration Makefile

BUILD_DIR := SAME70_nonRTOS
TARGET := $(BUILD_DIR)/libCANlib.a

SRC_DIR := src

CPP_SRCS := $(shell find $(SRC_DIR) -name '*.cpp')

INCLUDES := \
	-I$(SRC_DIR) \
	-I../RRFLibraries/src \
	-I../CoreN2G/src

DEFINES := \
	-D__SAME70Q20B__

CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m7 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
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
	-O2 \
	-Wall \
	-Werror \
	-Wnoexcept \
	-Wshadow \
	-Wsign-promo \
	$(INCLUDES) \
	$(DEFINES)

OBJS := $(CPP_SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

.PHONY: SAME70_nonRTOS
SAME70_nonRTOS: $(TARGET)

$(TARGET): $(OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MP -o $@ $<

-include $(DEPS)

.PHONY: clean-SAME70_nonRTOS
clean-SAME70_nonRTOS:
	$(Q)echo "  RM      $(BUILD_DIR)"
	$(Q)rm -rf $(BUILD_DIR)
