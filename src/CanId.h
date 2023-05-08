/*
 * CanId.h
 *
 *  Created on: 5 Jul 2019
 *      Author: David
 */

#ifndef SRC_CAN_CANID_H_
#define SRC_CAN_CANID_H_

#include <cstdint>
#include <CoreTypes.h>		// for CanAddress

// CAN message types. This is a 13-bit field, so we can use numbers 0 to 8191. Low numbers have highest priority.
enum class CanMessageType : uint16_t
{
	// High-priority requests sent by the main board
	emergencyStop = 0,
	startup = 10,
	controlledStop = 20,
	timeSync = 30,
	powerFailing = 40,
	stopMovement = 45,
	insertHiccup = 46,
	revertPosition = 47,
	//unused_was_movement = 50,
	movementLinear = 51,
	movementLinearShaped = 52,

	// High priority responses sent by expansion boards and Smart Tools
	inputStateChanged = 100,
	event = 102,
	enterTestMode = 104,	 	// sent by the ATE to the main board

	// Configuration messages sent by the main board
	setAddressAndNormalTiming = 2010,
	setFastTiming = 2011,
	reset = 2012,

	// Medium priority messages sent by the main board
	writeGpio = 4012,
	readInputsRequest = 4013,
	startAccelerometer = 4014,
	startClosedLoopDataCollection = 4015,

	// Configuration messages sent by the main board
	//unused_was_m950 = 6010,
	//unused_was_m308 = 6011,
	//unused_was_updateHeaterModelOld = 6012,
	setHeaterTemperature = 6013,
	//unused_was_setPressureAdvance = 6014,
	setDateTime = 6015,
	updateDeltaParameters = 6016,
	//unused_was_setMotorCurrents = 6017,
	m569 = 6018,
	fanParameters = 6019,
	m915 = 6020,
	//unused_was_setMicrostepping = 6021,
	//unused_was_setStandstillCurrentFactor = 6022,
	setDriverStates = 6023,
	returnInfo = 6024,
	updateFirmware = 6025,
	m950Heater = 6026,
	m950Fan = 6027,
	m950Gpio = 6028,
	setFanSpeed = 6029,
	setHeaterFaultDetection = 6030,
	m308New = 6031,
	heaterTuningCommand = 6032,
	heaterFeedForward = 6033,
	accelerometerConfig = 6034,
	m950Led = 6035,

	createInputMonitor = 6036,
	changeInputMonitor = 6037,
	acknowledgeAnnounce = 6038,
	setHeaterMonitors = 6039,
	diagnosticTest = 6040,

	m569p1 = 6041,
	setStepsPerMmAndMicrostepping = 6042,
	setMotorCurrents = 6043,
	setPressureAdvance = 6044,
	setStandstillCurrentFactor = 6045,
	createFilamentMonitor = 6046,
	deleteFilamentMonitor = 6047,
	configureFilamentMonitor = 6048,
	//unused_was_updateHeaterModelNew = 6049,
	m569p2 = 6050,
	m569p6 = 6051,
	m569p7 = 6052,
	heaterModelNewNew = 6053,
	setInputShaping = 6054,
	writeLedStrip = 6055,

	// Responses, broadcasts etc. sent by expansion boards
	standardReply = 4510,
	boardStatusReport = 4511,
	announceOld = 4512,							// announce message sent by firmware 3.4.0beta4 and earlier
	fanTachoReport = 4513,
	sensorTemperaturesReport = 4514,
	heatersStatusReport = 4515,
	//unused_was_fansRpmReport = 4516,			// replaced by fansReport
	fansReport = 4517,
	readInputsReply = 4518,
	driversStatusReport = 4519,
	filamentMonitorsStatusReport = 4520,
	heaterTuningReport = 4521,
	accelerometerData = 4522,
	closedLoopData = 4523,
	logMessage = 4524,
	announceNew = 4525,							// announce message sent by firmware 3.4.0beta5 and later
	closedLoopPIDStatus = 4526,

	// Firmware updates
	firmwareBlockRequest = 5000,
	firmwareBlockResponse = 5001,

	unusedMessageType = 0xFFFF
};

typedef uint16_t CanRequestId;
constexpr uint16_t CanRequestIdMask = 0x07FF;					// only the lower 12 bits used
constexpr CanRequestId CanRequestIdNoReplyNeeded = 0x0FFE;		// special ID means we don't want a reply
constexpr CanRequestId CanRequestIdAcceptAlways = 0x0FFF;		// special ID means always accept this

// CAN identifier
// A CAN identifier must identify
// 1. The sender, to make sure that 2 nodes can't both try to transmit at the same time.
// 2. The destination, so that each device can use the hardware ID filters in the MCU to receive only messages addressed to it, and broadcast messages.
// 3. The priority, so that if 2 nodes want to transmit at the same time, urgent messages can take precedence over less urgent messages.
// The CAN ID is transmitted most significant bit first. Logic level 0 is dominant. Therefore, lower IDs have greater priority.
// We use 29-bit CAN identifiers, split as for UAVCAN:
// Most significant 13 bits: message type/priority (lower is higher priority)
// Next 1 bit: response not request
// Next 7 bits: source address
// Next 1 bit: unused
// Lowest 7 bits: destination address
class CanId
{
	uint32_t all;

public:
	static constexpr CanAddress MasterAddress = 0;							// main boards (except ATE main boards) have address 0
	static constexpr CanAddress ATECMBoardFirstAddress = 90;
	static constexpr CanAddress ATEIOBoardFirstAddress = 95;
	static constexpr CanAddress ATECMBoardDefaultAddress = 118;
	static constexpr CanAddress ATEIOBoardDefaultAddress = 119;
	static constexpr CanAddress ToolBoardDefaultAddress = 121;				// default address for tool boards
	static constexpr CanAddress Exp1XDBoardDefaultAddress = 122;
	static constexpr CanAddress Exp1HCEBoardDefaultAddress = 123;
	static constexpr CanAddress SammyC21DefaultAddress = 124;
	static constexpr CanAddress ATEMasterAddress = 125;						// the address of the ATE main board
	static constexpr CanAddress ExpansionBoardFirmwareUpdateAddress = 126;	// special address we use for backup firmware update system (board ID switches set to zero on 3HC)
	static constexpr CanAddress MaxCanAddress = 126;						// maximum CAN address including the firmware update address
	static constexpr CanAddress BroadcastAddress = 127;
	static constexpr CanAddress NoAddress = 255;

	static constexpr uint32_t BoardAddressMask = 0x7F;
	static constexpr unsigned int DstAddressShift = 0;
	static constexpr unsigned int SrcAddressShift = 8;
	static constexpr uint32_t ResponseBit = 1ul << 15;
	static constexpr unsigned int MessageTypeShift = 16;
	static constexpr uint32_t MessageTypeMask = 0x1FFF;

	void SetRequest(CanMessageType msgType, CanAddress src, CanAddress dst) noexcept
	{
		all = ((uint32_t)msgType << MessageTypeShift) | ((uint32_t)src << SrcAddressShift) | ((uint32_t)dst << DstAddressShift);
	}

	void SetResponse(CanMessageType msgType, CanAddress src, CanAddress dst) noexcept
	{
		all = ((uint32_t)msgType << MessageTypeShift) | ((uint32_t)src << SrcAddressShift) | ((uint32_t)dst << DstAddressShift) | ResponseBit;
	}

	void SetBroadcast(CanMessageType msgType, CanAddress src) noexcept
	{
		all = ((uint32_t)msgType << MessageTypeShift) | ((uint32_t)src << SrcAddressShift) | ((uint32_t)BroadcastAddress << DstAddressShift);
	}

	void SetReceivedId(uint32_t id) noexcept
	{
		all = id;
	}

	uint8_t Src() const noexcept { return (all >> SrcAddressShift) & BoardAddressMask; }
	uint8_t Dst() const noexcept { return (all >> DstAddressShift) & BoardAddressMask; }
	CanMessageType MsgType() const noexcept { return (CanMessageType)((all >> MessageTypeShift) & MessageTypeMask); }
	uint32_t GetWholeId() const noexcept { return all; }
	bool IsRequest() const noexcept { return (all & ResponseBit) == 0; }
	bool IsResponse() const noexcept { return (all & ResponseBit) != 0; }

	bool operator==(CanId const& other) const noexcept
	{
		return all == other.all;
	}

	bool operator!=(CanId const& other) const noexcept
	{
		return !(*this == other);
	}
};

#endif /* SRC_CAN_CANID_H_ */
