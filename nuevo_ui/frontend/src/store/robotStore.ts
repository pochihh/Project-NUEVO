/**
 * Robot State Store (Zustand)
 * Single source of truth for all robot state.
 */
import { create } from 'zustand'
import type {
  ConnectionData,
  DCMotorItem,
  DCPidRspData,
  DCStateAllData,
  IMUData,
  IOInputStateData,
  IOOutputStateData,
  IOStatusData,
  KinematicsData,
  MagCalStatusData,
  SensorRangeData,
  SensorUltrasonicAllData,
  ServoStatusAllData,
  StepConfigRspData,
  StepStateAllData,
  StepperStatusItem,
  SysConfigRspData,
  SysDiagRspData,
  SysInfoRspData,
  SysPowerData,
  SysStateData,
  SystemStatusData,
  VoltageData,
} from '../lib/wsProtocol'

interface DCMotorState {
  status: DCMotorItem | null
  positionHistory: number[]
  velocityHistory: number[]
  currentHistory: number[]
  pwmHistory: number[]
  timeHistory: number[]
  frameIndexHistory: number[]
  recordingStartTs: number | null
}

export interface ErrorLogEntry {
  key: string
  label: string
  count: number
}

const SYSTEM_ERROR_LABELS: [number, string][] = [
  [0x01, 'Undervoltage'],
  [0x02, 'Overvoltage'],
  [0x04, 'Encoder Fail'],
  [0x08, 'I2C Error'],
  [0x10, 'IMU Error'],
]

const SYSTEM_WARNING_LABELS: [number, string][] = [
  [0x01, 'Liveness Lost'],
  [0x02, 'Loop Overrun'],
  [0x04, 'No Battery'],
  [0x08, 'Control Miss'],
]

const DC_FAULT_LABELS: [number, string][] = [
  [0x01, 'Overcurrent'],
  [0x02, 'Stall'],
]

const HISTORY_WINDOW_MS = 22_000
const BATTERY_PRESENT_MV = 2000
const BATTERY_UNDERVOLTAGE_CLEAR_MV = 10000

function addOrIncrement(log: ErrorLogEntry[], key: string, label: string): ErrorLogEntry[] {
  const idx = log.findIndex((e) => e.key === key)
  if (idx >= 0) {
    const next = [...log]
    next[idx] = { ...next[idx], count: next[idx].count + 1 }
    return next
  }
  return [...log, { key, label, count: 1 }]
}

function initDCMotors(): DCMotorState[] {
  return Array.from({ length: 4 }, () => ({
    status: null,
    positionHistory: [],
    velocityHistory: [],
    currentHistory: [],
    pwmHistory: [],
    timeHistory: [],
    frameIndexHistory: [],
    recordingStartTs: null,
  }))
}

function buildLegacyErrorFlags(sysState: SysStateData | null, sysPower: SysPowerData | null): number {
  if (!sysState) return 0
  let errorFlags = int(sysState.errorFlags)
  const batteryMv = int(sysPower?.batteryMv)

  // The live system header should follow the freshest power telemetry, not
  // an older sys_state packet that still carried a battery-related fault bit.
  if (batteryMv >= BATTERY_UNDERVOLTAGE_CLEAR_MV) {
    errorFlags &= ~0x01
  }

  return errorFlags
}

function int(value: number | undefined | null): number {
  return Number(value ?? 0)
}

function buildAttachedSensorsMask(sysInfo: SysInfoRspData | null): number {
  if (!sysInfo) return 0
  let mask = 0
  if (sysInfo.sensorCapabilityMask & 0x01) mask |= 0x01
  if (sysInfo.sensorCapabilityMask & 0x02) mask |= 0x04
  return mask
}

function buildSystemStatus(
  sysState: SysStateData | null,
  sysPower: SysPowerData | null,
  sysInfo: SysInfoRspData | null,
  sysConfig: SysConfigRspData | null,
  sysDiag: SysDiagRspData | null,
): SystemStatusData | null {
  if (!sysState) return null
  let warningFlags = int(sysState.warningFlags)
  const batteryMv = int(sysPower?.batteryMv)
  const errorFlags = buildLegacyErrorFlags(sysState, sysPower)
  if (batteryMv >= BATTERY_PRESENT_MV) {
    warningFlags &= ~0x04
  }
  return {
    firmwareMajor: int(sysInfo?.firmwareMajor),
    firmwareMinor: int(sysInfo?.firmwareMinor),
    firmwarePatch: int(sysInfo?.firmwarePatch),
    state: int(sysState.state),
    uptimeMs: int(sysState.uptimeMs),
    lastRxMs: int(sysState.lastRxMs),
    lastCmdMs: int(sysState.lastCmdMs),
    batteryMv: int(sysPower?.batteryMv),
    rail5vMv: int(sysPower?.rail5vMv),
    errorFlags,
    attachedSensors: buildAttachedSensorsMask(sysInfo),
    freeSram: int(sysDiag?.freeSram),
    loopTimeAvgUs: int(sysDiag?.loopTimeAvgUs),
    loopTimeMaxUs: int(sysDiag?.loopTimeMaxUs),
    uartRxErrors: int(sysDiag?.uartRxErrors),
    motorDirMask: int(sysConfig?.motorDirMask),
    neoPixelCount: int(sysConfig?.neoPixelCount),
    heartbeatTimeoutMs: int(sysConfig?.heartbeatTimeoutMs),
    limitSwitchMask: int(sysInfo?.limitSwitchMask),
    stepperHomeLimitGpio: sysInfo?.stepperHomeLimitGpio ?? [0xFF, 0xFF, 0xFF, 0xFF],
    warningFlags,
    runtimeFlags: int(sysState.runtimeFlags),
  }
}

function buildVoltage(sysPower: SysPowerData | null): VoltageData | null {
  if (!sysPower) return null
  return {
    batteryMv: int(sysPower.batteryMv),
    rail5vMv: int(sysPower.rail5vMv),
    servoRailMv: int(sysPower.servoRailMv),
  }
}

function buildIOStatus(ioInput: IOInputStateData | null, ioOutput: IOOutputStateData | null): IOStatusData | null {
  if (!ioInput && !ioOutput) return null
  return {
    buttonMask: int(ioInput?.buttonMask),
    limitMask: int(ioInput?.limitMask),
    ledBrightness: ioOutput?.ledBrightness ?? [0, 0, 0, 0, 0],
    timestamp: Math.max(int(ioInput?.timestamp), int(ioOutput?.timestamp)),
    neoPixels: ioOutput?.neoPixels ?? [],
  }
}

function buildRangeSensors(bundle: SensorUltrasonicAllData): SensorRangeData[] {
  const sensors: SensorRangeData[] = []
  for (let sensorId = 0; sensorId < Math.min(bundle.configuredCount, bundle.sensors.length); sensorId += 1) {
    const sensor = bundle.sensors[sensorId]
    sensors.push({
      sensorId,
      sensorType: 0,
      status: int(sensor.status),
      distanceMm: int(sensor.distanceMm),
      timestamp: int(bundle.timestamp),
    })
  }
  return sensors
}

function cacheKey(motorNumber: number, loopType: number): string {
  return `${motorNumber}:${loopType}`
}

function clearedRobotState(connection: ConnectionData | null, serialConnected: boolean): Partial<RobotState> {
  return {
    serialConnected,
    connection,
    system: null,
    voltage: null,
    dcMotors: initDCMotors(),
    steppers: [null, null, null, null],
    servo: null,
    io: null,
    kinematics: null,
    imu: null,
    magCal: null,
    rangeSensors: [],
    sysStateRaw: null,
    sysInfoRaw: null,
    sysConfigRaw: null,
    sysPowerRaw: null,
    sysDiagRaw: null,
    ioInputRaw: null,
    ioOutputRaw: null,
    dcPidCache: {},
    stepConfigCache: {},
  }
}

interface RobotState {
  connected: boolean
  serialConnected: boolean
  system: SystemStatusData | null
  voltage: VoltageData | null
  connection: ConnectionData | null
  dcMotors: DCMotorState[]
  steppers: (StepperStatusItem | null)[]
  servo: ServoStatusAllData | null
  io: IOStatusData | null
  kinematics: KinematicsData | null
  imu: IMUData | null
  magCal: MagCalStatusData | null
  rangeSensors: SensorRangeData[]
  errorLog: ErrorLogEntry[]
  warningLog: ErrorLogEntry[]
  sysStateRaw: SysStateData | null
  sysInfoRaw: SysInfoRspData | null
  sysConfigRaw: SysConfigRspData | null
  sysPowerRaw: SysPowerData | null
  sysDiagRaw: SysDiagRspData | null
  ioInputRaw: IOInputStateData | null
  ioOutputRaw: IOOutputStateData | null
  dcPidCache: Record<string, DCPidRspData>
  stepConfigCache: Record<number, StepConfigRspData>
  dispatch: (topic: string, data: any, ts?: number) => void
  setMotorRecording: (motorIdx: number, active: boolean) => void
  clearErrorLog: () => void
  clearWarningLog: () => void
}

export const useRobotStore = create<RobotState>((set) => ({
  connected: false,
  serialConnected: false,
  system: null,
  voltage: null,
  connection: null,
  dcMotors: initDCMotors(),
  steppers: [null, null, null, null],
  servo: null,
  io: null,
  kinematics: null,
  imu: null,
  magCal: null,
  rangeSensors: [],
  errorLog: [],
  warningLog: [],
  sysStateRaw: null,
  sysInfoRaw: null,
  sysConfigRaw: null,
  sysPowerRaw: null,
  sysDiagRaw: null,
  ioInputRaw: null,
  ioOutputRaw: null,
  dcPidCache: {},
  stepConfigCache: {},

  clearErrorLog: () => set({ errorLog: [] }),
  clearWarningLog: () => set({ warningLog: [] }),

  setMotorRecording: (motorIdx: number, active: boolean) => {
    set((state) => {
      const newDCMotors = [...state.dcMotors]
      const motor = { ...newDCMotors[motorIdx] }
      motor.recordingStartTs = active
        ? (motor.timeHistory[motor.timeHistory.length - 1] ?? null)
        : null
      newDCMotors[motorIdx] = motor
      return { dcMotors: newDCMotors }
    })
  },

  dispatch: (topic: string, data: any, ts?: number) => {
    switch (topic) {
      case 'sys_state':
        set((state) => {
          const sysStateRaw = data as SysStateData
          const prevSystem = state.system
          const system = buildSystemStatus(
            sysStateRaw,
            state.sysPowerRaw,
            state.sysInfoRaw,
            state.sysConfigRaw,
            state.sysDiagRaw,
          )

          let errorLog = state.errorLog
          let warningLog = state.warningLog
          const prevErrorFlags = prevSystem?.errorFlags ?? 0
          const prevWarningFlags = prevSystem?.warningFlags ?? 0
          if (system?.errorFlags) {
            for (const [bit, label] of SYSTEM_ERROR_LABELS) {
              if ((system.errorFlags & bit) && (prevErrorFlags & bit) === 0) {
                errorLog = addOrIncrement(errorLog, `sys_${bit}`, label)
              }
            }
          }
          if (system?.warningFlags) {
            for (const [bit, label] of SYSTEM_WARNING_LABELS) {
              if ((system.warningFlags & bit) && (prevWarningFlags & bit) === 0) {
                warningLog = addOrIncrement(warningLog, `warn_${bit}`, label)
              }
            }
          }

          const update: Partial<RobotState> = { sysStateRaw, system, errorLog, warningLog }
          const imuReady = (system?.runtimeFlags ?? 0) & 0x10
          if (!imuReady) {
            update.imu = null
          }
          if (system && (system.state === 3 || system.state === 4)) {
            update.dcMotors = state.dcMotors.map((m) => ({
              ...m,
              status: m.status ? { ...m.status, mode: 0, pwmOutput: 0, velocity: 0 } : null,
            }))
            update.steppers = state.steppers.map((s) =>
              s ? { ...s, enabled: 0, currentSpeed: 0 } : null
            )
            update.servo = state.servo
              ? {
                  ...state.servo,
                  channels: state.servo.channels.map((ch) => ({ ...ch, enabled: false })),
                }
              : null
          }
          return update
        })
        break

      case 'sys_power':
        set((state) => {
          const sysPowerRaw = data as SysPowerData
          return {
            sysPowerRaw,
            voltage: buildVoltage(sysPowerRaw),
            system: buildSystemStatus(
              state.sysStateRaw,
              sysPowerRaw,
              state.sysInfoRaw,
              state.sysConfigRaw,
              state.sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_info_rsp':
        set((state) => {
          const sysInfoRaw = data as SysInfoRspData
          return {
            sysInfoRaw,
            system: buildSystemStatus(
              state.sysStateRaw,
              state.sysPowerRaw,
              sysInfoRaw,
              state.sysConfigRaw,
              state.sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_config_rsp':
        set((state) => {
          const sysConfigRaw = data as SysConfigRspData
          return {
            sysConfigRaw,
            system: buildSystemStatus(
              state.sysStateRaw,
              state.sysPowerRaw,
              state.sysInfoRaw,
              sysConfigRaw,
              state.sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_diag_rsp':
        set((state) => {
          const sysDiagRaw = data as SysDiagRspData
          return {
            sysDiagRaw,
            system: buildSystemStatus(
              state.sysStateRaw,
              state.sysPowerRaw,
              state.sysInfoRaw,
              state.sysConfigRaw,
              sysDiagRaw,
            ),
          }
        })
        break

      case 'connection':
        set((state) => {
          const connection = data as ConnectionData
          const nextSerialConnected = connection.serialConnected
          const prevSerialConnected = state.serialConnected

          if (!nextSerialConnected || !prevSerialConnected) {
            return {
              ...clearedRobotState(connection, nextSerialConnected),
            }
          }

          return {
            connection,
            serialConnected: nextSerialConnected,
          }
        })
        break

      case 'dc_pid_rsp':
        set((state) => {
          const incoming = data as DCPidRspData
          const nextCache = {
            ...state.dcPidCache,
            [cacheKey(incoming.motorNumber, incoming.loopType)]: incoming,
          }
          const idx = incoming.motorNumber - 1
          const nextMotors = [...state.dcMotors]
          const prev = nextMotors[idx]
          if (prev?.status) {
            const status = { ...prev.status }
            if (incoming.loopType === 0) {
              status.posKp = incoming.kp
              status.posKi = incoming.ki
              status.posKd = incoming.kd
            } else {
              status.velKp = incoming.kp
              status.velKi = incoming.ki
              status.velKd = incoming.kd
            }
            nextMotors[idx] = { ...prev, status }
          }
          return { dcPidCache: nextCache, dcMotors: nextMotors }
        })
        break

      case 'dc_state_all': {
        const bridgeTimeMs = ts ? ts * 1000 : Date.now()
        const cutoff = bridgeTimeMs - HISTORY_WINDOW_MS
        const motors = (data as DCStateAllData).motors

        for (const motor of motors) {
          if (motor.faultFlags) {
            set((state) => {
              let errorLog = state.errorLog
              for (const [bit, suffix] of DC_FAULT_LABELS) {
                if (motor.faultFlags & bit) {
                  errorLog = addOrIncrement(errorLog, `dc_${bit}_${motor.motorNumber}`, `Motor ${motor.motorNumber} ${suffix}`)
                }
              }
              return { errorLog }
            })
          }
        }

        set((state) => {
          const newDCMotors = [...state.dcMotors]
          for (const motor of motors) {
            const idx = motor.motorNumber - 1
            if (idx < 0 || idx > 3) continue

            const posPid = state.dcPidCache[cacheKey(motor.motorNumber, 0)]
            const velPid = state.dcPidCache[cacheKey(motor.motorNumber, 1)]
            const merged: DCMotorItem = {
              ...motor,
              posKp: posPid?.kp ?? 0,
              posKi: posPid?.ki ?? 0,
              posKd: posPid?.kd ?? 0,
              velKp: velPid?.kp ?? 0,
              velKi: velPid?.ki ?? 0,
              velKd: velPid?.kd ?? 0,
            }

            const prev = newDCMotors[idx]
            const isRecording = prev.recordingStartTs !== null
            const newTime = [...prev.timeHistory, bridgeTimeMs]
            const newPos = [...prev.positionHistory, merged.position]
            const newVel = [...prev.velocityHistory, merged.velocity]
            const newCur = [...prev.currentHistory, merged.currentMa]
            const newPwm = [...prev.pwmHistory, merged.pwmOutput]
            const newFrame = [...prev.frameIndexHistory, merged.frameIndex]

            let start = 0
            if (!isRecording) {
              while (start < newTime.length && newTime[start] < cutoff) start += 1
            }

            newDCMotors[idx] = {
              status: merged,
              recordingStartTs: prev.recordingStartTs,
              positionHistory: start > 0 ? newPos.slice(start) : newPos,
              velocityHistory: start > 0 ? newVel.slice(start) : newVel,
              currentHistory: start > 0 ? newCur.slice(start) : newCur,
              pwmHistory: start > 0 ? newPwm.slice(start) : newPwm,
              timeHistory: start > 0 ? newTime.slice(start) : newTime,
              frameIndexHistory: start > 0 ? newFrame.slice(start) : newFrame,
            }
          }
          return { dcMotors: newDCMotors }
        })
        break
      }

      case 'step_config_rsp':
        set((state) => {
          const incoming = data as StepConfigRspData
          const nextCache = { ...state.stepConfigCache, [incoming.stepperNumber]: incoming }
          const idx = incoming.stepperNumber - 1
          const nextSteppers = [...state.steppers]
          const prev = nextSteppers[idx]
          if (prev) {
            nextSteppers[idx] = {
              ...prev,
              maxSpeed: incoming.maxVelocity,
              acceleration: incoming.acceleration,
            }
          }
          return { stepConfigCache: nextCache, steppers: nextSteppers }
        })
        break

      case 'step_state_all':
        set((state) => {
          const next = [...state.steppers]
          for (const s of (data as StepStateAllData).steppers) {
            const idx = s.stepperNumber - 1
            if (idx < 0 || idx > 3) continue
            const cfg = state.stepConfigCache[s.stepperNumber]
            next[idx] = {
              ...s,
              commandedCount: s.count,
              limitHit: s.limitFlags,
              maxSpeed: cfg?.maxVelocity ?? 0,
              acceleration: cfg?.acceleration ?? 0,
            }
          }
          return { steppers: next }
        })
        break

      case 'servo_state_all':
        set({ servo: data as ServoStatusAllData })
        break

      case 'io_input_state':
        set((state) => {
          const ioInputRaw = data as IOInputStateData
          return {
            ioInputRaw,
            io: buildIOStatus(ioInputRaw, state.ioOutputRaw),
          }
        })
        break

      case 'io_output_state':
        set((state) => {
          const ioOutputRaw = data as IOOutputStateData
          return {
            ioOutputRaw,
            io: buildIOStatus(state.ioInputRaw, ioOutputRaw),
          }
        })
        break

      case 'sensor_kinematics':
        set({ kinematics: data as KinematicsData })
        break

      case 'sensor_imu':
        set({ imu: data as IMUData })
        break

      case 'sensor_mag_cal_status':
        set({ magCal: data as MagCalStatusData })
        break

      case 'sensor_ultrasonic_all':
        set({ rangeSensors: buildRangeSensors(data as SensorUltrasonicAllData) })
        break

      default:
        console.log('[Store] Unknown topic:', topic, data)
    }
  },
}))
