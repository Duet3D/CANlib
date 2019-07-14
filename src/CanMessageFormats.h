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
// Generic message
struct CanMessageGeneric
{
	uint8_t data[64];
};

// Time sync message
struct CanMessageTimeSync
{
	static constexpr CanMessageType messageType = CanMessageType::timeSync;

	uint32_t timeSent;									// when this message was sent
	uint32_t lastTimeSent;								// when we tried to send the previous message
	uint32_t lastTimeAcknowledged;						// when the previous message was acknowledged
};

// Emergency stop message
struct CanMessageEmergencyStop
{
	static constexpr CanMessageType messageType = CanMessageType::emergencyStop;
};

// Movement message
struct CanMessageMovement
{
	static constexpr CanMessageType messageType = CanMessageType::movement;

	uint32_t whenToExecute;
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;

	uint32_t deltaDrives : 4,				// which drivers are doing delta movement
			 pressureAdvanceDrives : 4,		// which drivers have pressure advance applied
			 endStopsToCheck : 4,			// which drivers have endstop checks applied
			 stopAllDrivesOnEndstopHit : 1;	// whether to stop all drivers when one endstop is hit

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
	uint16_t heaterNumber;
	uint16_t spare;
	float setPoint;
};

struct CanMessageM303
{
	uint16_t heaterNumber;
};

struct CanMessageM305
{
	uint16_t heaterNumber;
	uint16_t validParams;
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

	bool GotParamB() const { return IsBitSet(validParams, 0); }
	bool GotParamC() const { return IsBitSet(validParams, 1); }
	bool GotParamD() const { return IsBitSet(validParams, 2); }
	bool GotParamF() const { return IsBitSet(validParams, 3); }
	bool GotParamL() const { return IsBitSet(validParams, 4); }
	bool GotParamH() const { return IsBitSet(validParams, 5); }
	bool GotParamR() const { return IsBitSet(validParams, 6); }
	bool GotParamT() const { return IsBitSet(validParams, 7); }
	bool GotParamW() const { return IsBitSet(validParams, 8); }
};

#define M305_SET_IF_PRESENT(_val, _msg, _letter) if (_msg.GotParam ## _letter ()) { _val = _msg.param ## _letter; }

struct CanMessageM307
{
	uint16_t heaterNumber;
	//TODO
};

struct CanMessageM350
{
	uint32_t whichDrives;
	struct MicrostepSetting
	{
		uint32_t microstepping : 16,
				 interpolated : 1;
	};
	MicrostepSetting motorMicrostepping[MaxDriversPerCanSlave];
};

struct CanMessageM569
{
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
	uint32_t whichDrives;
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
	uint32_t whichDrives;
	uint32_t motorCurrentPercentages[MaxDriversPerCanSlave];
};

struct CanMessageM915
{
	uint32_t whichDrives;
	struct DriverStallSetting
	{
		//TODO
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
	uint32_t status;						// 2 bits per heater: off, on, or fault
	struct HeaterStatus
	{
		int16_t actualTemp;
		float setPoint;
	};
	HeaterStatus heaterStatus[MaxHeatersPerCanSlave];
};

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
