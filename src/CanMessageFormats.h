/*
 * CanMessageFormats.h
 *
 *  Created on: 16 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEFORMATS_H_
#define SRC_CAN_CANMESSAGEFORMATS_H_

#include "CanId.h"
#include "General/BitMap.h"

constexpr unsigned int MaxDriversPerCanSlave = 4;
constexpr unsigned int MaxHeatersPerCanSlave = 6;

// CAN message formats
// We have maximum 64 bytes in a CAN-FD packet.
// The first 4 bytes are the time to start the command, in clock ticks. This field doubles up as a sequence number.
// So the CAN packets sent by the master must be in strict increasing order. If 2 commands in separate packets need to be executed at the same time,
// the second must be delayed by 1 clock tick.
// Some commands are always executed immediately on receipt. For these, the first 4 bytes are still present and serve as a sequence number.

// The next 4 bytes are split as follows:
//  10 bit command number. For many types of command (but not movement commands), this is the corresponding M-code number.
//   6 bit flags. The meaning depends on the command. Unused bits must be set to zero for future compatibility.
//  16 bit map of which of the following command parameters are valid.

// The remaining up to 56 bytes are organised as up to 14 parameters. Each one is either a float, an int32_t or a uint32_t.

// Movement messages may be a little different because of the need to carry a lot of data, however the first dword will still be the start time and it will
// be possible to distinguish a movement command by looking at the next 16 bits.

// This is the layout of the second dword
struct CanMessageHeader
{
	uint16_t code : 10,
			 codeFlags : 6;
	union
	{
		// Usual meaning of this word
		uint16_t validParams;

		// Meaning of this word in a movement message
		uint16_t deltaDrives : 4,				// which drivers are doing delta movement
				 pressureAdvanceDrives : 4,		// which drivers have pressure advance applied
				 endStopsToCheck : 4,			// which drivers have endstop checks applied
				 stopAllDrivesOnEndstopHit : 1;	// whether to stop all drivers when one endstop is hit
	} u;
};

// Generic message
struct CanMessageGeneric
{
	CanMessageHeader hdr;
	uint8_t data[60];
};

// Time sync message
struct CanMessageTimeSync
{
	static constexpr CanMessageType messageType = CanMessageType::timeSync;

	CanMessageHeader hdr;
	uint32_t timeSent;									// when this message was sent
};

// Emergency stop message
struct CanMessageEmergencyStop
{
	static constexpr CanMessageType messageType = CanMessageType::emergencyStop;

	CanMessageHeader hdr;
};

// Movement message
struct CanMessageMovement
{
	static constexpr CanMessageType messageType = CanMessageType::movement;

	CanMessageHeader hdr;
	uint32_t whenToExecute;
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;
	float initialSpeedFraction;
	float finalSpeedFraction;

	float initialX;						// needed only for delta movement
	float initialY;						// needed only for delta movement
	float finalX;						// needed only for delta movement
	float finalY;						// needed only for delta movement
	float zMovement;					// needed only for delta movement

	struct
	{
		int32_t steps;					// net steps moved
	} perDrive[MaxDriversPerCanSlave];

	void DebugPrint();
};


// Here are the layouts for standard message types, excluding the first dword.

struct CanMessageM140								// also does M104, M141
{
	CanMessageHeader hdr;
	float setPoint[MaxHeatersPerCanSlave];
};

struct CanMessageM303
{
	CanMessageHeader hdr;
	uint32_t heaterNumber;
};

struct CanMessageM305
{
	CanMessageHeader hdr;
	uint32_t heaterNumber;
	uint32_t channel;
	float paramB;
	float paramC;
	int32_t paramD;
	int32_t paramF;
	float paramL;
	float paramH;
	float paramR;
	float paramT;
	uint32_t paramW;

	bool GotParamB() const { return IsBitSet(hdr.u.validParams, 0); }
	bool GotParamC() const { return IsBitSet(hdr.u.validParams, 1); }
	bool GotParamD() const { return IsBitSet(hdr.u.validParams, 2); }
	bool GotParamF() const { return IsBitSet(hdr.u.validParams, 3); }
	bool GotParamL() const { return IsBitSet(hdr.u.validParams, 4); }
	bool GotParamH() const { return IsBitSet(hdr.u.validParams, 5); }
	bool GotParamR() const { return IsBitSet(hdr.u.validParams, 6); }
	bool GotParamT() const { return IsBitSet(hdr.u.validParams, 7); }
	bool GotParamW() const { return IsBitSet(hdr.u.validParams, 8); }
};

#define M305_SET_IF_PRESENT(_val, _msg, _letter) if (_msg.GotParam ## _letter ()) { _val = _msg.param ## _letter; }

struct CanMessageM307
{
	CanMessageHeader hdr;

};

struct CanMessageM350
{
	CanMessageHeader hdr;
	struct MicrostepSetting
	{
		uint32_t microstepping : 16,
				 interpolated : 1;
	};
	MicrostepSetting motorMicrostepping[MaxDriversPerCanSlave];
};

struct CanMessageM569
{
	CanMessageHeader hdr;
	uint32_t driverNumber;
	uint32_t driverMode : 3,
			 direction : 1,
			 enablePolarity : 1,
			 stepPulseLength : 5,
			 stepPulseInterval : 5,
			 dirSetupTime : 5,
			 dirHoldTime : 5,
			 tpfd : 4,
			 blankingTime : 2,
			 offTime : 4,
			 hstart : 3,
			 hend : 4,
			 hdec : 2,
			 tpwmthrs : 20,
			 thigh : 20;
};

struct CanMessageM906
{
	CanMessageHeader hdr;
	struct MotorSetting
	{
		uint16_t currentMilliamps;
		uint8_t standstillCurrentPercent;
		uint8_t idleCurrentPercent;
	};
	MotorSetting motorCurrents[MaxDriversPerCanSlave];
};

struct CanMessageM913
{
	CanMessageHeader hdr;
	uint32_t motorCurrentPercentages[MaxDriversPerCanSlave];
};

struct CanMessageM915
{
	CanMessageHeader hdr;
	struct DriverStallSetting
	{

	};
	DriverStallSetting stallSettings[MaxDriversPerCanSlave];
};

union CanMessage
{
	CanMessageGeneric generic;
	CanMessageTimeSync sync;
	CanMessageEmergencyStop eStop;
	CanMessageMovement move;
	CanMessageM140 m140;
	CanMessageM303 m303;
	CanMessageM305 m305;
	CanMessageM307 m307;
	CanMessageM350 m350;
	CanMessageM569 m569;
	CanMessageM906 m906;
	CanMessageM913 m913;
	CanMessageM915 m915;
};

static_assert(sizeof(CanMessage) <= 64, "CAN message too big");		// check none of the messages is too large

// Standard responses

struct CanResponseHeaterStatus
{
	CanMessageHeader hdr;
	uint32_t status;						// 2 bits per heater: off, on, or fault
	struct HeaterStatus
	{
		int16_t actualTemp;
		float setPoint;
	};
	HeaterStatus heaterStatus[MaxHeatersPerCanSlave];
};

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
