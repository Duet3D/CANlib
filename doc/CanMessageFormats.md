# CAN message formats

Reference for every message in the `CanMessage` union defined in
[`CANlib/src/CanMessageFormats.h`](../src/CanMessageFormats.h). The numeric
message type / priority for each is in [`CANlib/src/CanId.h`](../src/CanId.h).

For the high-level protocol description (speeds, time sync, assumptions, message
categories) see [`Duet3CAN-FDProtocol.md`](./Duet3CAN-FDProtocol.md).

## How to read this document

"Master" = the main board (CAN address `0`). "Expansion board" / "slave" = any
remote board (tool board, expansion board, Smart Tool…). Most descriptions below
are written from the master's point of view; in *test/expansion mode*
(`SUPPORT_REMOTE_COMMANDS`) a main board can also act as a slave and process the
same request messages — see `CommandProcessor::ProcessReceivedMessage` in
[`src/CAN/CommandProcessor.cpp`](../../../src/CAN/CommandProcessor.cpp).

In the parameter tables the **Type** column gives the C type; `:N` denotes an
`N`-bit bitfield. Fields named `zero`/`zeroN` are spare padding (set to 0) and are
omitted. Sizes such as `MaxLinearDriversPerCanSlave` come from `RRF3Common.h`.

### The three transport patterns

1. **Request / standard reply.** The master fills in a request, allocates a
   `CanRequestId` and calls
   `CanInterface::SendRequestAndGetStandardReply()`
   ([`src/CAN/CanInterface.cpp`](../../../src/CAN/CanInterface.cpp)). It then
   blocks (default `UsualResponseTimeout = 1000 ms`) waiting for a
   `CanMessageStandardReply` whose `requestId` matches. The reply carries a
   `GCodeResult` result code, optional 8-bit `extra`, and text that can be split
   over several fragments; the text is concatenated into the caller's `reply`
   `StringRef`, the result code is returned, and `extra` is copied out. On
   timeout `GCodeResult::canResponseTimeout` is returned. The slave builds the
   reply in the common tail of `ProcessReceivedMessage` (the
   `SetupResponseMessage<CanMessageStandardReply>` loop).

2. **Request / custom reply.** Same as above but the master calls
   `SendRequestAndGetCustomReply()` naming an expected reply message type and a
   callback. A `CanMessageStandardReply` is still accepted (e.g. on error,
   matched by `requestId` which is in the same place in every reply struct);
   otherwise the named reply type is delivered to the callback. Used by
   `readInputsRequest` and `setDefaultHeaterModel`.

3. **Fire-and-forget.** No reply is matched. This covers:
   - motion / urgent messages sent through the dedicated Tx FIFO via
     `SendMotion()` / `SendMessageNoReplyNoFree()`;
   - broadcasts (`SendBroadcastNoFree()`), such as time sync and all the periodic
     status reports;
   - requests sent with `requestId == CanRequestIdNoReplyNeeded`.

`CanRequestIdNoReplyNeeded` and `CanRequestIdAcceptAlways` are the two special
request IDs (see `CanId.h`).

> Several union members are **not** message structs in their own right — they are
> the element types of `CanMessageMultipleDrivesRequest<T>` or sub-records of a
> larger message (`CanSensorReport`, `CanHeaterReport`, `FanReport`,
> `AnalogHandleDataV0/V1`, `OpenLoopStatus`, `ClosedLoopStatus`,
> `FilamentMonitorDataV2`, `StepsPerUnitAndMicrostepping`, `DriverStateControl`).
> They are documented inline with their containing message.
> `CanMessageM303` is defined in the header but is **not** a union member.

---

## Housekeeping, time and lifecycle

### `CanMessageTimeSync` (`sync`, type `timeSync` = 30)

| Field | Type | Description |
|---|---|---|
| `timeSent` | `uint32_t` | When this message was sent |
| `lastTimeSent` | `uint32_t` | When we tried to send the previous message |
| `lastTimeAcknowledgeDelay` | `uint32_t:16` | Delay before the previous message was acknowledged |
| `isPrinting` | `:1` | Set while printing, so filament monitors collect data |
| `fastDataRate` | `:3` | CAN-FD data bit rate ÷ nominal − 1; 0 = don't use BRS |
| `tseg1Minus1` | `:8` | tseg1 value for the data phase, minus 1 |
| `realTime` | `uint32_t` | Epoch seconds (UTC). Present from RRF 3.2 only |
| `movementDelay` | `uint32_t` | Cumulative hiccup time. Not always present |

Valid lengths: `SizeWithoutRealTime` / `SizeWithRealTime` /
`SizeWithRealTimeAndMovementDelay`.
- **Where used:** broadcast periodically by the master `CAN_CLOCK` task
  (`StepTimer`, `CanInterface`), every `CanClockIntervalMillis = 211 ms`. Sent
  without BRS, through a dedicated buffer. Slaves run a frequency-locked loop off
  it (`StepTimer::ProcessTimeSyncMessage`) and use it to negotiate the fast data
  rate (`CanInterface::CheckBrs`).
- **Reply:** none (broadcast).

### `CanMessageEmergencyStop` (`eStop`, type `emergencyStop` = 0)

No parameters (highest-priority message).
- **Where used:** send to every CAN address individually, then as a broadcast by the master (e.g. `ExpansionManager`) on emergency
  stop. Slave calls `reprap.EmergencyStop()` then `ScheduleReset()`.
- **Reply:** none.

### `CanMessageReset` (`reset`, type `reset` = 2012)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |

- **Where used:** master tells a board to reset. Slave acknowledges, then
  emergency-stops and reschedules a reset.
- **Reply:** standard reply ("Board *n* resetting"), handled per pattern 1.

### `CanMessageEnterTestMode` (`enterTestMode`, type `enterTestMode` = 104)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `address` | `uint16_t:7` | CAN address to adopt |
| `passwd` | `uint32_t` | Integrity check; must equal `Passwd = 0x57a82fd1` |

- **Where used:** sent by the ATE to make a main board behave as an expansion
  board. On a correct password the board sends its reply *then* switches to
  expansion mode (`SwitchToExpansionMode`).
- **Reply:** standard reply (empty text), sent before the mode switch.

### `CanMessageAnnounceV0` (`announceV0`, type `announceV0` = 4512)

| Field | Type | Description |
|---|---|---|
| `timeSinceStarted` | `uint32_t` | Milliseconds since the board started |
| `numDrivers` | `uint32_t:8` | Number of motor drivers on the board |
| `boardTypeAndFirmwareVersion` | `char[56]` | `"<shortName>|<version>"` |

Firmware ≤ 3.4.0beta4.
- **Where used:** broadcast by a starting expansion board until acknowledged.
  Master: `ExpansionManager::ProcessAnnouncement(buf, false)`.
- **Reply:** the master replies with `acknowledgeAnnounce` (to the sending board), not a
  standard reply.

### `CanMessageAnnounceV1` (`announceV1`, type `announceV1` = 4525)

| Field | Type | Description |
|---|---|---|
| `timeSinceStarted` | `uint32_t` | Milliseconds since the board started |
| `uniqueId` | `uint8_t[16]` | Board's unique ID |
| `numDrivers` | `uint8_t:4` | Number of motor drivers on the board |
| `usesUf2Binary` | `:1` | Set if main firmware is taken in `.uf2` format |
| `boardTypeAndFirmwareVersion` | `char[43]` | `"<shortName>|<version>"` |

Firmware ≥ 3.4.0beta5.
- **Where used:** as `announceV0` but `ProcessAnnouncement(buf, true)`.
- **Reply:** acknowledged with `acknowledgeAnnounce`.

### `CanMessageAcknowledgeAnnounce` (`acknowledgeAnnounce`, type `acknowledgeAnnounce` = 6038)

No parameters.
- **Where used:** broadcast by the master in response to an announce. Slave calls
  `CanInterface::MainBoardAcknowledgedAnnounce()` to stop announcing.
- **Reply:** none.

### `CanMessageSetAddressAndNormalTiming` (`setAddressAndNormalTiming`, type = 2010)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `oldAddress` | `uint8_t` | Current CAN address of the target board |
| `newAddress` | `uint8_t` | Address to assign |
| `newAddressInverted` | `uint8_t` | Complement of `newAddress` (integrity check) |
| `doSetTiming` | `uint8_t` | `DoSetTimingYes` (0xB6) to write timing, else `DoSetTimingNo` |
| `normalTiming` | `CanTiming` | Normal-rate bit-timing parameters |

Built by `CanInterface::ChangeAddressAndNormalTiming` (M952). Does not use BRS.
- **Where used:** master changes a board's CAN address and/or normal data rate.
- **Reply:** standard reply, handled per pattern 1.

---

## Motion

### `CanMessageMovementLinearShaped` (`moveLinearShaped`, type `movementLinearShaped` = 52)

| Field | Type | Description |
|---|---|---|
| `whenToExecute` | `uint32_t` | Master step-clock time at which the move starts |
| `accelerationClocks` | `uint32_t` | Duration of the acceleration phase (clocks) |
| `steadyClocks` | `uint32_t` | Duration of the steady-speed phase (clocks) |
| `decelClocks` | `uint32_t` | Duration of the deceleration phase (clocks) |
| `extruderDrives` | `uint32_t:8` | Bitmap of which drivers are extruders |
| `numDrivers` | `:4` | Number of drivers included (≤ 8) |
| `seq` | `:4` | Sequence number (`SeqMask = 0x0f`) |
| `usePressureAdvance` | `:1` | Apply PA to extruders and accumulate partial steps |
| `useLateInputShaping` | `:1` | Apply input shaping late |
| `acceleration` | `float` | Base acceleration, distance normalised to 1.0 |
| `deceleration` | `float` | Base deceleration, distance normalised to 1.0 |
| `perDrive[]` | `PerDriveValues` | Per-driver `int32_t steps` (motors) / `float extrusion` (extruders) |

`GetActualDataLength()` / `HasMotion()` helpers.
- **Where used:** built per board by `CanMotion` and sent through the motion Tx
  FIFO via `SendMotion()`. Slave: duplicate/out-of-sequence detection on `seq`,
  then `Move::AddMoveFromRemote()` (only if `StepTimer::IsSynced()`).
- **Reply:** none. Duplicates and out-of-order frames are counted in master
  diagnostics (`CommandProcessor::AppendBadMotionStats`).

### `CanMessageStopMovement` (`stopMovement`, type `stopMovement` = 45)

| Field | Type | Description |
|---|---|---|
| `whichDrives` | `uint16_t` | Bitmap of drives to stop; `0xFFFF` = all on the board |

- **Where used:** urgent message from `CanMotion` to abort moves. Slave:
  `Move::StopDriversFromRemote()`.
- **Reply:** none.

### `CanMessageRevertPosition` (`revertPosition`, type `revertPosition` = 47)

| Field | Type | Description |
|---|---|---|
| `whichDrives` | `uint32_t:16` | Bitmap of drivers whose step counts are included |
| `clocksAllowed` | `uint32_t` | Step clocks allowed for the revert move |
| `finalStepCounts[]` | `int32_t[MaxLinearDriversPerCanSlave]` | Net steps of the last move to revert |

`GetActualDataLength(numReverting)`.
- **Where used:** after an endstop/Z-probe-triggered stop, master tells the board
  the net step counts to revert to (`CanMotion`, `Move::RevertPosition`). Slave:
  `Move::RevertPosition()`.
- **Reply:** none.

---

## Generic forwarded G/M-codes — `CanMessageGeneric` (`generic`)

### `CanMessageGeneric`

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint32_t:12` | Request ID echoed in the reply |
| `paramMap` | `:20` | Bitmap of which parameters are present, in `ParamTable` order |
| `data` | `uint8_t[60]` | Parameters packed per a `ParamTable` known to both ends |

See `CanMessageGenericConstructor` / `CanMessageGenericParser`.
- **Where used:** the master serializes many M-codes into this one structure and
  sends them via the request/reply path. Message types that travel as a
  `CanMessageGeneric`: `m950Heater`, `m950Fan`, `m950Gpio`, `m950Led`,
  `writeLedStrip`, `m308V1`, `m915`, `m569`, `m569p2`, `m569p7`,
  `configureFilamentMonitor`, `m655`, `m111`. The slave dispatches on the CAN
  message *type* (not on the struct) to the relevant handler
  (`Heat`, `FansManager`, `Platform`/LED, `Move`, `FilamentMonitor`, …).
- **Reply:** standard reply, per pattern 1 (some carry `extra`, e.g. M950 LED).

---

## Heaters

### `CanMessageSetHeaterTemperatureV1` (`setTemp`, type `setHeaterTemperatureV1` = 6068, RRF 3.7)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `heaterNumber` | `uint16_t:8` | Heater to control |
| `function` | `:3` | Heater function |
| `setPoint` | `float` | Target temperature |
| `command` | `uint8_t:4` | `commandNone`/`Off`/`On`/`ResetFault`/`Suspend`/`Unsuspend`/`Reset` |

- **Where used:** master sets a remote heater set-point / state. Slave:
  `Heat::SetTemperature()`.
- **Reply:** standard reply.

### `CanMessageHeaterModelV3` (`heaterModelV3`, type `heaterModelV3` = 6069, RRF 3.7)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `heater` | `uint16_t:8` | Heater number |
| `enabled` | `:1` | Heater model enabled |
| `inverted` | `:1` | Output inverted |
| `basicModel` | `HeaterModel` | The heater model |
| `maxPwm` | `float` | Maximum PWM |

Several `_obsolete_was_*` PID fields are retained for layout only.
- **Where used:** master pushes a heater model (M307). Slave: `Heat::ProcessM307()`.
- **Reply:** standard reply.

### `CanMessageSetHeaterFaultDetectionParameters` (`setHeaterFaultDetection`, type `setHeaterFaultDetection` = 6030)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `version35` | `:1` | Set if `maxBadTemperatureCount` is present (RRF 3.5+) |
| `heater` | `uint16_t` | Heater number |
| `maxTempExcursion` | `float` | Permitted temperature excursion |
| `maxFaultTime` | `float` | Permitted fault time |
| `maxBadTemperatureCount` | `uint32_t` | Added 3.5; valid only if `version35` |

M570.
- **Where used:** `Heat::SetFaultDetection()`.
- **Reply:** standard reply.

### `CanMessageSetHeaterMonitors` (`setHeaterMonitors`, type `setHeaterMonitors` = 6039)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `numMonitors` | `:4` | Number of monitors in `monitors[]` |
| `heater` | `uint16_t` | Heater number |
| `monitors[]` | `CanHeaterMonitor[7]` | Each: `float limit`, `int8_t sensor`, `uint8_t action`, `int8_t trigger` |

`GetActualDatalength()`.
- **Where used:** `Heat::SetHeaterMonitors()`.
- **Reply:** standard reply.

### `CanMessageHeaterTuningCommand` (`heaterTuningCommand`, type `heaterTuningCommand` = 6032)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `heaterNumber` | `uint32_t:8` | Heater being tuned |
| `on` | `:1` | Whether the heater is on for this half-cycle |
| `pwm` | `float` | Tuning PWM |
| `lowTemp` | `float` | Lower cycling temperature |
| `highTemp` | `float` | Upper cycling temperature |
| `peakTempDrop` | `float` | Allowed peak temperature drop |

Drives M303 auto-tune cycling.
- **Where used:** `Heat::TuningCommand()`.
- **Reply:** standard reply. Per-cycle results come back separately as
  `heaterTuningReport` messages (below).

### `CanMessageHeaterFeedForwardV1` (`heaterFeedForwardV1`, type `heaterFeedForwardV1` = 6063)

| Field | Type | Description |
|---|---|---|
| `heaterNumber` | `uint16_t:8` | Heater number (**no `requestId`**) |
| `fanPwmFraction` | `float` | Fan PWM feed-forward fraction |
| `extrusionPwmBoost` | `float` | PWM boost while extruding |
| `extrusionTemperatureBoost` | `float` | Temperature boost while extruding |

- **Where used:** master sends feed-forward parameters. Slave:
  `Heat::ApplyFeedForward()`.
- **Reply:** none — the handler sets `requestId = CanRequestIdNoReplyNeeded`, so
  no reply is sent and the master does not wait.

### `CanMessageSetDefaultHeaterModel` (`setDefaultHeaterModel`, type `setDefaultHeaterModel` = 6067, RRF 3.7)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `heater` | `uint16_t:6` | Heater number |
| `heaterFunction` | `:3` | Heater function |

- **Where used:** master asks a board to set and return the default model for a
  heater (`RemoteHeater`). Slave: `Heat::SetDefaultHeaterModel()`.
- **Reply:** **custom** — `CanMessageHeaterModelReport`, via
  `SendRequestAndGetCustomReply`.

### `CanMessageHeaterModelReport` (`heaterModelReport`, type `heaterModelReport` = 4531, RRF 3.7)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint32_t:12` | Request ID being replied to (same place as in a standard reply) |
| `resultCode` | `:4` | `GCodeResult` (same place as in a standard reply) |
| `heaterNumber` | `:6` | Heater reported |
| `model` | `HeaterModel` | The returned model |

- **Where used:** slave's reply to `setDefaultHeaterModel`; built by
  `LocalHeater`. Delivered to the master's custom-reply callback, which copies the
  returned model into the local `RemoteHeater`.
- **Reply:** n/a (this *is* the reply).

---

## Fans

### `CanMessageFanParameters` (`fanParameters`, type `fanParameters` = 6019)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `fanNumber` | `uint16_t` | Fan number |
| `blipTime` | `uint16_t` | Blip time (milliseconds) |
| `val` | `float` | Current fan value |
| `minVal` | `float` | Minimum value |
| `maxVal` | `float` | Maximum value |
| `triggerTemperatures` | `float[2]` | Thermostatic trigger temperatures |
| `sensorsMonitored` | `uint64_t` | Bitmap of sensors driving the fan |

M106 config.
- **Where used:** `FansManager::ConfigureFan()`.
- **Reply:** standard reply.

### `CanMessageSetFanSpeed` (`setFanSpeed`, type `setFanSpeed` = 6029)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `fanNumber` | `uint16_t` | Fan number |
| `pwm` | `float` | Requested PWM |

- **Where used:** `FansManager::SetFanSpeed()`.
- **Reply:** standard reply.

### `CanMessageFansReport` (`fansReport`, type `fansReport` = 4517)

| Field | Type | Description |
|---|---|---|
| `whichFans` | `uint64_t` | Bitmap of fan numbers reported |
| `fanReports[]` | `FanReport[14]` | Each: `uint16_t actualPwm`, `int16_t rpm` (−1 if no tacho) |

`GetActualDataLength(numReported)`.
- **Where used:** broadcast periodically by a board. Master:
  `FansManager::ProcessRemoteFanRpms()`. Not logged/flashed as activity.
- **Reply:** none (status broadcast).

---

## GPIO / servo

### `CanMessageWriteGpio` (`writeGpio`, type `writeGpio` = 4012)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `isServo` | `:1` | Treat the port as a servo |
| `pwm` | `float` | PWM / servo value |
| `portNumber` | `uint8_t` | GPIO port number |

M42 / M280.
- **Where used:** `CanInterface::WriteGpio` → `Platform::EutHandleGpioWrite()`.
- **Reply:** standard reply.

---

## Input monitors / endstops / Z-probes

### `CanMessageCreateInputMonitorV1` (`createInputMonitorV1`, type = 6060)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `handle` | `RemoteInputHandle` | Handle to create |
| `threshold` | `int32_t` | Analog threshold, or 0 if digital |
| `minInterval` | `uint16_t` | Minimum reporting interval |
| `pinName` | `char[54]` | Null-terminated pin name |

`GetActualDataLength()`.
- **Where used:** `CanInterface::CreateHandle` → slave `InputMonitor::Create()`.
- **Reply:** standard reply; `extra` returns the current state.

### `CanMessageChangeInputMonitorV1` (`changeInputMonitorV1`, type = 6061)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `handle` | `RemoteInputHandle` | Handle to change |
| `param` | `uint32_t` | Action-specific parameter (see below) |
| `action` | `uint8_t` | `actionDontMonitor`/`DoMonitor`/`Delete`/`ChangeThreshold`/`ChangeMinInterval`/`ReturnPinName`/`SetDriveLevel`/`SelectTouchMode` |

For `actionSetDriveLevel`, `param` has special encodings for scanning Z-probes
(auto-calibrate / report drive level, or drive level + offset).
- **Where used:** `CanInterface` handle helpers (enable, delete, change threshold/
  interval, get pin name, drive level, touch mode) → `InputMonitor::Change()`.
- **Reply:** standard reply; `extra` returns current state, text returns the pin
  name for `actionReturnPinName`.

### `CanMessageEnableStallEndstop` (`enableStallEndstop`, type `enableStallEndstop` = 6065)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `driverNumber` | `uint16_t` | Driver to enable a stall endstop for; `disableAll = 0xFFFF` |
| `speed` | `float` | Homing-move speed (ignored if `disableAll`) |

- **Where used:** `CanInterface::EnableRemoteStallEndstop` /
  `DisableRemoteStallEndstops` → `Move::SetStallEndstopReporting()`.
- **Reply:** standard reply.

### `CanMessageReadInputsRequest` (`readInputsRequest`, type `readInputsRequest` = 4013)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint32_t:12` | Request ID echoed in the reply |
| `mask` | `RemoteInputHandle` | Mask applied when matching handles |
| `pattern` | `RemoteInputHandle` | Handle pattern to match |

- **Where used:** `CanInterface::ReadRemoteHandles` →
  `InputMonitor::ReadInputs(buf)`.
- **Reply:** **custom** — `CanMessageReadInputsReplyV0` (or `…V1`), via
  `SendRequestAndGetCustomReply`; the callback delivers each handle/reading pair.

### `CanMessageReadInputsReplyV0` (`readInputsReplyV0`, type `readInputsReplyV0` = 4518)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint32_t:12` | Request ID being replied to |
| `resultCode` | `:4` | `GCodeResult` |
| `numReported` | `:4` | Number of handles reported |
| `results[]` | `AnalogHandleDataV0[10]` | Each: `RemoteInputHandle handle`, `int32_t reading` (unaligned) |

- **Reply payload for `readInputsRequest`.** Consumed by the read-handles callback.

### `CanMessageReadInputsReplyV1` (`readInputsReplyV1`, type `readInputsReplyV1` = 4529)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint32_t:12` | Request ID being replied to |
| `resultCode` | `:4` | `GCodeResult` |
| `numReported` | `:4` | Number of handles reported |
| `results[]` | `AnalogHandleDataV1[7]` | Each: `handle`, `uint16_t when`, `int32_t reading` (aligned) |

- **Reply payload for `readInputsRequest`** (newer boards).

### `CanMessageInputChangedV1` (`inputChangedV1`, type `inputStateChangedV1` = 105)

| Field | Type | Description |
|---|---|---|
| `states` | `uint16_t` | 1 bit per reported handle (current state) |
| `numHandles` | `uint8_t` | Number of entries in `results[]` |
| `results[]` | `AnalogHandleDataV0[10]` | Each: `handle`, `int32_t reading` |

`AddEntry`/`GetEntry*` helpers.
- **Where used:** **sent by the slave** when a monitored input changes. Master:
  `HandleInputStateChangedV1()` routes per handle type to endstops, Z-probe,
  GpIn, or stall-endstop handling, then `Move::OnEndstopOrZProbeStatesChanged()`.
- **Reply:** none (asynchronous notification). Legacy; kept for older firmware.

### `CanMessageInputChangedV2` (`inputChangedV2`, type `inputStateChangedV2` = 106)

| Field | Type | Description |
|---|---|---|
| `states` | `uint16_t` | 1 bit per reported handle (current state) |
| `numHandles` | `uint8_t` | Number of entries in `results[]` |
| `results[]` | `AnalogHandleDataV1[7]` | Each: `handle`, `uint16_t when`, `int32_t reading` |

`GetWhen()` accessor.
- **Where used / reply:** as V1 but `HandleInputStateChangedV2()`, using the
  per-entry timestamp instead of the buffer timestamp. Current version.

---

## Multiple-driver configuration — `CanMessageMultipleDrivesRequest<T>`

One templated struct, instantiated several ways in the union.

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `driversToUpdate` | `uint16_t` | Bitmap of drivers to update |
| `values[]` | `T[MaxLinearDriversPerCanSlave]` | One value of type `T` per set bit |

`GetActualDataLength(numDrivers)` / `MaxDrivesPerMessage()`. The instantiations:

| Union member | `T` | CAN type | Meaning / slave handler |
|---|---|---|---|
| `multipleDrivesRequestUint16` | `uint16_t` | `setDriverStates` (6023) | Driver state per `DriverStateControl` (disabled/idle/active + brake/idle timing) → `Move::EutHandleSetDriverStates` |
| `multipleDrivesRequestFloat` | `float` | `setMotorCurrents` (6043) / `setStandstillCurrentFactor` (6045) / `setPressureAdvanceV1` (6044) | motor currents (mA) / standstill % / PA → `Move::EutSetMotorCurrents` etc. |
| `multipleDrivesStepsPerUnitAndMicrostepping` | `StepsPerUnitAndMicrostepping` | `setStepsPerMmAndMicrostepping` (6042) | `float stepsPerUnit` + `uint16_t microstepping`(+interp) → `Move::EutSetStepsPerMmAndMicrostepping` |
| `multipleDrivesRequestDriverState` | `DriverStateControl` | `setDriverStates` (6023) | typed view used by the handler |
| `multipleDrivesRequestPressureAdvance` | `ShortPressureAdvanceParameters` | `setPressureAdvanceV2` (6070) | PA params → `Move::EutSetRemotePressureAdvanceV2` |

- **Where used:** `CanInterface` driver helpers (`EnableRemoteDrivers`,
  `SetRemoteDriverCurrents`, `SetRemoteStandstillCurrentPercent`,
  `SetRemotePressureAdvance`, `SetRemoteDriverStepsPerMmAndMicrostepping`, …),
  grouping drivers by board with `CanDriversData`.
- **Reply:** standard reply.

### `CanMessageSetInputShapingV1` (`setInputShapingV1`, type `setInputShapingV1` = 6062)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `numImpulses` | `uint16_t` | Total number of impulses |
| `impulses[]` | `ShapingPair[7]` | Each: `float coefficient`, `uint32_t impulseDelay` |

`GetActualDataLength()`.
- **Where used:** `AxisShaper` pushes the configured shaper to each board →
  `Move::EutSetInputShaping()`.
- **Reply:** standard reply.

---

## Filament monitors

### `CanMessageCreateFilamentMonitor` (`createFilamentMonitor`, type `createFilamentMonitor` = 6046)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `driver` | `uint16_t:8` | Driver the monitor is associated with |
| `type` | `:8` | Filament monitor type |

M591.
- **Where used:** `CanInterface::CreateFilamentMonitor` → `FilamentMonitor::Create()`.
- **Reply:** standard reply. (Configuration is sent separately as a `generic`
  `configureFilamentMonitor`.)

### `CanMessageDeleteFilamentMonitor` (`deleteFilamentMonitor`, type `deleteFilamentMonitor` = 6047)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `driver` | `uint16_t:8` | Driver whose monitor is deleted |

- **Where used:** `CanInterface::DeleteFilamentMonitor` → `FilamentMonitor::Delete()`
  (called from a destructor, so must not throw).
- **Reply:** standard reply.

### `CanMessageFilamentMonitorsStatusV2` (`filamentMonitorsStatusV2`, type `filamentMonitorsStatusReportV2` = 4528)

| Field | Type | Description |
|---|---|---|
| `driversReported` | `uint32_t:8` | Bitmap of drivers with monitors reported |
| `data[]` | `FilamentMonitorDataV2[5]` | Per monitor: `position`, `status`, `hasLiveData`, min/max/avg/last percentages, `calibrationLength` |

`GetActualDataLength()`.
- **Where used:** broadcast by a board. Master:
  `FilamentMonitor::UpdateRemoteFilamentStatus()`.
- **Reply:** none (status broadcast).

---

## Diagnostics, info and test

### `CanMessageReturnInfo` (`getInfo`, type `returnInfo` = 6024)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `param` | `:4` | M122 P parameter |
| `type` | `uint8_t` | `typeFirmwareVersion`/`typeBoardName`/`typeBootloaderName`/`typeBoardUniqueId`/`typeDiagnosticsPart0…` |

- **Where used:** `CanInterface::GetRemoteFirmwareDetails` /
  `RemoteDiagnostics` → slave `EutGetInfo()`.
- **Reply:** standard reply; text carries the requested info, `extra` carries the
  number of remaining diagnostics parts so the master can page through them.

### `CanMessageDiagnosticTest` (`diagnosticTest`, type `diagnosticTest` = 6040)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `testType` | `uint16_t` | M122 P parameter |
| `invertedTestType` | `uint16_t` | Complement of `testType` (integrity check) |
| `param16` | `uint16_t` | Optional 16-bit parameter |
| `param32` | `uint32_t[3]` | Optional 32-bit parameters |

- **Where used:** `CanInterface::RemoteDiagnostics` for test sub-functions (the
  master may not get a reply if the test crashes/resets the board).
- **Reply:** standard reply when one is produced.

### `CanMessageM303`

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID |
| `heaterNumber` | `uint16_t` | Heater number |
| `targetTemperature` | `float` | Target temperature |

Defined in the header but **not** a member of the `CanMessage` union — auto-tune
is driven via `CanMessageHeaterTuningCommand`.

---

## Firmware update

### `CanMessageUpdateYourFirmware` (`updateYourFirmware`, type `updateFirmware` = 6025)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `module` | `:2` | 0 = main firmware, 1 = bootloader (2,3 reserved) |
| `boardId` | `uint8_t` | Target board ID |
| `invertedBoardId` | `uint8_t` | Complement of `boardId` (integrity check) |

- **Where used:** master tells a board to start updating. Slave:
  `InitiateFirmwareUpdate()` validates the address/module, checks the IAP binary
  exists, then `ScheduleFirmwareUpdateOverCan()`.
- **Reply:** standard reply ("Board *n* starting firmware update" or an error).

### `CanMessageFirmwareUpdateRequest` (`firmwareUpdateRequest`, type `firmwareBlockRequest` = 5000)

| Field | Type | Description |
|---|---|---|
| `fileOffset` | `uint32_t:24` | Offset in the file of the data needed |
| `bootloaderVersion` | `:5` | Protocol version of the requester (currently 0) |
| `uf2Format` | `:1` | Set if UF2 format wanted, else binary |
| `fileWanted` | `:2` | 0 = firmware, 3 = bootloader |
| `lengthRequested` | `uint32_t:24` | How much data is wanted |
| `boardVersion` | `:8` | Hardware version of the board |
| `boardType` | `char[56]` | Board type / bootloader class name (no `requestId`) |

- **Where used:** **sent by the board being updated** (bootloader/firmware) to ask
  the master for a chunk of the firmware/bootloader file. Master:
  `HandleFirmwareBlockRequest()` opens `Duet3Firmware_<type>.bin/.uf2`
  (or `Duet3Bootloader-…`) from SD/SBC.
- **Reply:** one or more `CanMessageFirmwareUpdateResponse` messages
  (`SendResponseNoFree`), looping until the requested length is satisfied;
  completion / failure drives `ExpansionManager::UpdateFinished/UpdateFailed`.

### `CanMessageFirmwareUpdateResponse` (`firmwareUpdateResponse`, type `firmwareBlockResponse` = 5001)

| Field | Type | Description |
|---|---|---|
| `fileOffset` | `uint32_t:24` | Offset in the file where this block starts |
| `dataLength` | `:6` | Number of valid bytes in `data` (≤ 56) |
| `err` | `:2` | `ErrNone`/`ErrNoFile`/`ErrBadOffset`/`ErrOther` |
| `fileLength` | `uint32_t:24` | Total size of the firmware file |
| `data` | `uint8_t[56]` | Up to 56 bytes of file data |

- **Where used:** the master's response to `firmwareUpdateRequest` (above).
- **Reply:** n/a (this *is* the response).

---

## Status / data broadcasts from boards (no reply)

These are sent by boards (broadcast or to the master) and consumed by master
handlers. None expects a reply; most are not treated as "activity" (no LED flash)
in `ProcessReceivedMessage`.

### `CanMessageSensorTemperatures` (`sensorTemperaturesBroadcast`, type `sensorTemperaturesReport` = 4514)

| Field | Type | Description |
|---|---|---|
| `whichSensors` | `uint64_t` | Bitmap of sensor numbers reported |
| `temperatureReports[]` | `CanSensorReport[11]` | Each: `uint8_t errorCode` (`TemperatureError`), `float temperature` |

`GetActualDataLength(numSensors)`.
- **Master handler:** `Heat::ProcessRemoteSensorsReport()`.

### `CanMessageHeatersStatus` (`heatersStatusBroadcast`, type `heatersStatusReport` = 4515)

| Field | Type | Description |
|---|---|---|
| `whichHeaters` | `uint64_t` | Bitmap of heater numbers reported |
| `reports[]` | `CanHeaterReport[9]` | Each: `uint8_t mode` (`HeaterMode`), `uint8_t averagePwm` (0-255), `float temperature` |

- **Master handler:** `Heat::ProcessRemoteHeatersReport()`.

### `CanMessageHeaterTuningReport` (`heaterTuningReport`, type `heaterTuningReport` = 4521)

| Field | Type | Description |
|---|---|---|
| `heater` | `uint32_t:8` | Heater number |
| `cyclesDone` | `:16` | Tuning cycles completed |
| `ton` / `toff` | `uint32_t` | Heater on / off times |
| `dlow` / `dhigh` | `uint32_t` | Dead times at low / high temperature |
| `heatingRate` | `float` | Measured heating rate |
| `coolingRate` | `float` | Measured cooling rate |
| `voltage` | `float` | Supply voltage during the cycle |

- **Where used:** sent at the end of each M303 tuning cycle.
- **Master handler:** `Heat::ProcessRemoteHeaterTuningReport()`.

### `CanMessageBoardStatusV0` (`boardStatusV0`, type `boardStatusReportV0` = 4511)

| Field | Type | Description |
|---|---|---|
| `hasVin`/`hasV12`/`hasMcuTemp` | `:1` each | Which `MinCurMax` values are present |
| `hasAccelerometer`/`hasClosedLoop`/`hasInductiveSensor` | `:1` each | Board capability flags |
| `hasMovementDelay` | `:1` | Selects the union field below |
| `numAnalogHandles` | `:3` | Number of trailing `AnalogHandleDataV0` records |
| `neverUsedRam` / `movementDelay` | `int32_t` / `uint32_t` (union) | RAM low-water mark, or movement delay if `hasMovementDelay` |
| `values[]` | `MinCurMax[3]` | None/some/all of Vin, V12, CPU temperature |
| *(trailing)* | `AnalogHandleDataV0[]` | Up to `numAnalogHandles` analog handle readings |

Offset/length helpers (`GetAnalogHandlesOffset()` etc.).
- **Master handler:** `ExpansionManager::ProcessBoardStatusReport()`.

### `CanMessageBoardStatusV1` (`boardStatusV1`, type `boardStatusReportV1` = 4530)

| Field | Type | Description |
|---|---|---|
| *(flags)* | as V0 | Same capability/selection flags as V0 |
| `numAnalogHandles` | `:3` | Number of trailing `AnalogHandleDataV1` records |
| `neverUsedRam` / `movementDelay` | `int32_t` / `uint32_t` (union) | As V0 |
| `shortValues[]` | `ShortMinCurMax[3]` | None/some/all of Vin, V12, CPU temperature (compact) |
| *(trailing)* | `AnalogHandleDataV1[]` | Up to `numAnalogHandles` analog handle readings (more fit than V0) |

- **Master handler:** `ExpansionManager::ProcessBoardStatusReport()`.

### `CanMessageDriversStatus` (`driversStatus`, type `driversStatusReport` = 4519)

| Field | Type | Description |
|---|---|---|
| `numDriversReported` | `uint16_t:4` | Number of drivers reported |
| `hasClosedLoopData` | `:1` | Selects which union arm is used |
| `openLoopData[]` | `OpenLoopStatus[15]` | Per driver: `uint32_t status` (when not closed loop) |
| `closedLoopData[]` | `ClosedLoopStatus[5]` | Per driver: `status` + `float16_t` current/position-error fields (when closed loop) |

`SetStandardFields()` / `GetActualDataLength()`.
- **Master handler:** `ExpansionManager::ProcessDriveStatusReport()`.

### `CanMessageEvent` (`event`, type `event` = 102)

| Field | Type | Description |
|---|---|---|
| `eventType` | `uint32_t:8` | What happened |
| `deviceNumber` | `:8` | Device it happened to |
| `eventParam` | `:16` | More info about the event |
| `text` | `char[56]` | Human-readable detail |

- **Where used:** sent by a board to raise an event
  (`CanInterface::RaiseEvent`). Master: `Event::Add()`.

### `CanMessageDebugText` (`debugText`, type `debugText` = 4526)

| Field | Type | Description |
|---|---|---|
| `text` | `char[64]` | Debug text to display |

- **Where used:** debug text from a board; master prints it via
  `Platform::MessageF` ("Debug from *n*: …").

### `CanMessageAccelerometerData` (`accelerometerData`, type `accelerometerData` = 4522)

| Field | Type | Description |
|---|---|---|
| `actualSampleRate` | `uint32_t:14` | Measured sample rate (0 if not yet measured) |
| `numSamples` | `:6` | Samples in this packet (per requested axis) |
| `overflowed` | `:1` | Accelerometer detected overflow |
| `axes` | `:3` | Which axes are present |
| `bitsPerSampleMinusOne` | `:4` | Bits per sample, minus one |
| `lastPacket` | `:1` | Set on the final packet |
| `firstSampleNumber` | `uint16_t` | Number of the first sample |
| `data` | `uint16_t[29]` | Packed sample data |

`SetAxesAndResolution()` / `GetActualDataLength()`.
- **Where used:** streamed by a board after `startAccelerometer`. Master:
  `Accelerometers::ProcessReceivedData()` (writes the CSV/data file).

### `CanMessageClosedLoopData` (`closedLoopData`, type `closedLoopData` = 4523)

| Field | Type | Description |
|---|---|---|
| `numSamples` | `uint32_t:5` | Samples in this packet |
| `lastPacket` | `:1` | Set on the final packet |
| `filter` | `:16` | Which variables are present |
| `overflowed` | `:1` | Buffer overflow occurred |
| `badSample` | `:1` | A bad sample occurred (should not happen) |
| `firstSampleNumber` | `uint32_t:20` | Number of the first sample |
| `data` | `uint8_t[56]` | Packed sample data |

`GetActualDataLength()` / `GetNumDataBytes()`.
- **Where used:** streamed by a board after `startClosedLoopDataCollection`.
  Master: `ClosedLoop::ProcessReceivedData()`.

---

## Collecting accelerometer / closed-loop data (requests)

### `CanMessageStartAccelerometer` (`startAccelerometer`, type `startAccelerometer` = 4014)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `deviceNumber` | `uint8_t` | Accelerometer device |
| `axes` | `uint8_t:3` | Bitmap of axes to collect |
| `delayedStart` | `:1` | Delay collection until `startTime` |
| `numSamples` | `uint32_t` | How many samples to collect |
| `startTime` | `uint32_t` | Step-timer ticks to start at (if `delayedStart`) |

- **Where used:** `CanInterface::StartAccelerometer`. Data then streams back as
  `accelerometerData` broadcasts.
- **Reply:** standard reply (acknowledging the start).

### `CanMessageStartClosedLoopDataCollection` (`startClosedLoopDataCollection`, type = 4015)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint16_t:12` | Request ID echoed in the reply |
| `rate` | `uint16_t` | Sample rate |
| `filter` | `uint16_t` | Which variables to collect |
| `deviceNumber` | `uint8_t` | Device to collect for |
| `mode` | `uint8_t` | Collection mode |
| `numSamples` | `uint16_t` | How many samples to collect |
| `movement` | `uint8_t` | Which (if any) movement was requested |

- **Where used:** `CanInterface::StartClosedLoopDataCollection`. Data then streams
  back as `closedLoopData` broadcasts.
- **Reply:** standard reply.

---

## Common reply type

### `CanMessageStandardReply` (`standardReply`, type `standardReply` = 4510)

| Field | Type | Description |
|---|---|---|
| `requestId` | `uint32_t:12` | Request ID being replied to |
| `resultCode` | `:4` | `GCodeResult` |
| `fragmentNumber` | `:7` | Fragment number of this message |
| `moreFollows` | `:1` | Set if this is not the last fragment |
| `extra` | `:8` | Usually unused; occasionally carries extra data |
| `text` | `char[60]` | Reply text (`MaxTextLength`) |

`GetTextLength()` / `GetActualDataLength()`.
- **Where used:** the universal reply to request/response messages, built in
  `ProcessReceivedMessage`'s common tail and sent (possibly fragmented) via
  `SendResponseNoFree`. The master accumulates the fragments in
  `SendRequestAndGetStandardReply`/`…CustomReply`, returns `resultCode`, and copies
  out `extra`. Long replies are split across fragments with `moreFollows` set.

---

*Generated from `CanMessageFormats.h`, `CanId.h`, `CanInterface.{h,cpp}`,
`CommandProcessor.cpp` and the relevant module handlers.*
