/**
 * WebSocket Protocol Type Definitions
 * Matches nuevo_bridge message_router.py exactly.
 */

// ============================================================================
// Envelope
// ============================================================================

export interface WSMessage {
  topic: string
  data: any
  ts: number
}

export interface WSCommand {
  cmd: string
  data: Record<string, any>
}

// ============================================================================
// Incoming — System
// ============================================================================

export interface SystemStatusData {
  firmwareMajor: number
  firmwareMinor: number
  firmwarePatch: number
  state: number          // 0=INIT 1=IDLE 2=RUNNING 3=ERROR 4=ESTOP
  uptimeMs: number
  lastRxMs: number
  lastCmdMs: number
  batteryMv: number
  rail5vMv: number
  errorFlags: number
  attachedSensors: number
  freeSram: number
  loopTimeAvgUs: number
  loopTimeMaxUs: number
  uartRxErrors: number
  wheelDiameterMm: number
  wheelBaseMm: number
  motorDirMask: number
  neoPixelCount: number
  heartbeatTimeoutMs: number
  limitSwitchMask: number
  stepperHomeLimitGpio: number[]
}

// ============================================================================
// Incoming — Voltage
// ============================================================================

export interface VoltageData {
  batteryMv: number
  rail5vMv: number
  servoRailMv: number
}

// ============================================================================
// Incoming — Connection stats
// ============================================================================

export interface ConnectionData {
  serialConnected: boolean
  port: string
  baud: number
  rxCount: number
  txCount: number
  crcErrors: number
}

// ============================================================================
// Incoming — DC Motors (dc_status_all topic)
// ============================================================================

export interface DCMotorItem {
  motorNumber: number   // 1-based (converted by bridge)
  frameIndex: number    // sequential counter incremented per DC_STATUS_ALL packet
  mode: number          // 0=disabled 1=position 2=velocity 3=pwm
  faultFlags: number
  position: number      // ticks
  velocity: number      // ticks/s
  targetPos: number
  targetVel: number
  pwmOutput: number     // -255 to +255
  currentMa: number
  posKp: number; posKi: number; posKd: number
  velKp: number; velKi: number; velKd: number
}

export interface DCStatusAllData {
  motors: DCMotorItem[]
}

// ============================================================================
// Incoming — Stepper Motors (step_status_all topic)
// ============================================================================

export interface StepperStatusItem {
  stepperNumber: number  // 1-based
  enabled: number        // 0 or 1
  motionState: number    // 0=idle 1=accel 2=cruise 3=decel 4=homing 5=fault
  limitHit: number       // bitmask: bit0=min bit1=max
  commandedCount: number
  targetCount: number
  currentSpeed: number   // steps/s
  maxSpeed: number
  acceleration: number
}

export interface StepperStatusAllData {
  steppers: StepperStatusItem[]
}

// ============================================================================
// Incoming — Servos (servo_status_all topic)
// ============================================================================

export interface ServoChannelItem {
  channelNumber: number  // 1-based
  enabled: boolean
  pulseUs: number        // 0 if disabled
}

export interface ServoStatusAllData {
  pca9685Connected: number
  pca9685Error: number
  channels: ServoChannelItem[]
}

// ============================================================================
// Incoming — User I/O (io_status topic)
// ============================================================================

export interface IOStatusData {
  buttonMask: number           // bit N = button N+1 (0-based mask, 1-based UI)
  ledBrightness: number[]      // [led0, led1, led2]
  timestamp: number
  neoPixels?: { r: number; g: number; b: number }[]
}

// ============================================================================
// Incoming — Kinematics (kinematics topic)
// ============================================================================

export interface KinematicsData {
  x: number      // mm
  y: number      // mm
  theta: number  // rad
  vx: number     // mm/s
  vy: number     // mm/s
  vTheta: number // rad/s
  timestamp: number
}
