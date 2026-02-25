/**
 * Robot State Store (Zustand)
 * Single source of truth for all robot state.
 */
import { create } from 'zustand'
import type {
  SystemStatusData,
  VoltageData,
  ConnectionData,
  DCStatusAllData,
  DCMotorItem,
  StepperStatusAllData,
  StepperStatusItem,
  ServoStatusAllData,
  IOStatusData,
  KinematicsData,
} from '../lib/wsProtocol'

interface DCMotorState {
  status: DCMotorItem | null
  positionHistory: number[]
  velocityHistory: number[]
  currentHistory: number[]
  pwmHistory: number[]
  timeHistory: number[]        // bridge receive time in ms (ts * 1000)
  frameIndexHistory: number[]  // sequential frameIndex from bridge
  recordingStartTs: number | null  // bridge time ms when recording started; null = not recording
}

interface RobotState {
  connected: boolean
  serialConnected: boolean
  system: SystemStatusData | null
  voltage: VoltageData | null
  connection: ConnectionData | null
  dcMotors: DCMotorState[]          // indexed 0-3 (motorNumber - 1)
  steppers: (StepperStatusItem | null)[]  // indexed 0-3
  servo: ServoStatusAllData | null
  io: IOStatusData | null
  kinematics: KinematicsData | null
  dispatch: (topic: string, data: any, ts?: number) => void
  setMotorRecording: (motorIdx: number, active: boolean) => void
}

const HISTORY_WINDOW_MS = 22_000

const initDCMotors = (): DCMotorState[] =>
  Array.from({ length: 4 }, () => ({
    status: null,
    positionHistory: [],
    velocityHistory: [],
    currentHistory: [],
    pwmHistory: [],
    timeHistory: [],
    frameIndexHistory: [],
    recordingStartTs: null,
  }))

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

  setMotorRecording: (motorIdx: number, active: boolean) => {
    set((state) => {
      const newDCMotors = [...state.dcMotors]
      const motor = { ...newDCMotors[motorIdx] }
      if (active) {
        // recordingStartTs = bridge time of the most recent sample (or null if no data yet)
        const lastTs = motor.timeHistory[motor.timeHistory.length - 1] ?? null
        motor.recordingStartTs = lastTs
      } else {
        motor.recordingStartTs = null
      }
      newDCMotors[motorIdx] = motor
      return { dcMotors: newDCMotors }
    })
  },

  dispatch: (topic: string, data: any, ts?: number) => {
    switch (topic) {
      case 'system_status': {
        const newSystem = data as SystemStatusData
        set({ system: newSystem })

        // When ESTOP/ERROR is detected, immediately reflect disabled state in the UI
        // without waiting for individual motor/servo/stepper status updates.
        if (newSystem.state === 4 || newSystem.state === 3) {
          set((state) => ({
            // Mark all DC motors as mode=0 (disabled)
            dcMotors: state.dcMotors.map((m) => ({
              ...m,
              status: m.status ? { ...m.status, mode: 0, pwmOutput: 0, velocity: 0 } : null,
            })),
            // Mark all steppers as disabled
            steppers: state.steppers.map((s) =>
              s ? { ...s, enabled: 0, currentSpeed: 0 } : null
            ),
            // Mark all servo channels as disabled
            servo: state.servo
              ? {
                  ...state.servo,
                  channels: state.servo.channels.map((ch) => ({ ...ch, enabled: false })),
                }
              : null,
          }))
        }
        break
      }

      case 'voltage':
        set({ voltage: data as VoltageData })
        break

      case 'connection':
        set({
          connection: data as ConnectionData,
          serialConnected: (data as ConnectionData).serialConnected,
        })
        break

      case 'dc_status_all': {
        // Use bridge receive time (ts * 1000 ms) for accurate timing
        const bridgeTimeMs = ts ? ts * 1000 : Date.now()
        const cutoff = bridgeTimeMs - HISTORY_WINDOW_MS
        const motors = (data as DCStatusAllData).motors

        set((state) => {
          const newDCMotors = [...state.dcMotors]
          for (const motor of motors) {
            const idx = motor.motorNumber - 1
            if (idx < 0 || idx > 3) continue
            const prev = newDCMotors[idx]
            const isRecording = prev.recordingStartTs !== null

            const newTime  = [...prev.timeHistory, bridgeTimeMs]
            const newPos   = [...prev.positionHistory, motor.position]
            const newVel   = [...prev.velocityHistory, motor.velocity]
            const newCur   = [...prev.currentHistory, motor.currentMa]
            const newPwm   = [...prev.pwmHistory, motor.pwmOutput]
            const newFrame = [...prev.frameIndexHistory, motor.frameIndex]

            // Only prune the rolling window when NOT recording
            let start = 0
            if (!isRecording) {
              while (start < newTime.length && newTime[start] < cutoff) start++
            }

            newDCMotors[idx] = {
              status: motor,
              recordingStartTs: prev.recordingStartTs,
              positionHistory:    start > 0 ? newPos.slice(start)   : newPos,
              velocityHistory:    start > 0 ? newVel.slice(start)   : newVel,
              currentHistory:     start > 0 ? newCur.slice(start)   : newCur,
              pwmHistory:         start > 0 ? newPwm.slice(start)   : newPwm,
              timeHistory:        start > 0 ? newTime.slice(start)  : newTime,
              frameIndexHistory:  start > 0 ? newFrame.slice(start) : newFrame,
            }
          }
          return { dcMotors: newDCMotors }
        })
        break
      }

      case 'step_status_all': {
        const steppers = (data as StepperStatusAllData).steppers
        set((state) => {
          const next = [...state.steppers]
          for (const s of steppers) {
            const idx = s.stepperNumber - 1
            if (idx >= 0 && idx <= 3) next[idx] = s
          }
          return { steppers: next }
        })
        break
      }

      case 'servo_status_all':
        set({ servo: data as ServoStatusAllData })
        break

      case 'io_status':
        set({ io: data as IOStatusData })
        break

      case 'kinematics':
        set({ kinematics: data as KinematicsData })
        break

      default:
        console.log('[Store] Unknown topic:', topic, data)
    }
  },
}))
