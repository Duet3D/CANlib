# CANlib SAMC21_nonRTOS Configuration Makefile

BUILD_DIR := SAMC21_nonRTOS
TARGET := $(BUILD_DIR)/libCANlib.a

SRC_DIR := src

CPP_SRCS := $(shell find $(SRC_DIR) -name '*.cpp')

INCLUDES := \
	-I$(SRC_DIR) \
	-I../RRFLibraries/src \
	-I../CoreN2G/src

DEFINES := \
	-D__SAMC21G18A__

CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m0plus \
	-mthumb \
	-fno-math-errno \
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

.PHONY: SAMC21_nonRTOS
SAMC21_nonRTOS: $(TARGET)

$(TARGET): $(OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MP -o $@ $<

-include $(DEPS)

.PHONY: SAMC21_nonRTOS
clean-SAMC21_nonRTOS:
	$(Q)echo "  RM      $(BUILD_DIR)"
	$(Q)rm -rf $(BUILD_DIR)
