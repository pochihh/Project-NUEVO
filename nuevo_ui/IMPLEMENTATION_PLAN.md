# NUEVO UI - Implementation Plan

## Context

We need a web-based dashboard to monitor and control the NUEVO board robotics platform. The UI runs on the Raspberry Pi 5 and connects to the Arduino Mega 2560 via UART using the existing TLV protocol. It must work **without ROS2** (standalone mode), with ROS2 integration added later as an optional plugin. When ROS2 is running, other nodes (e.g., `robot_control_node`) may also send commands through the bridge — the UI can still monitor everything.

## Architecture

```
Arduino ←(UART/TLV @ 500kbaud)→ nuevo_bridge (Python) ←(WebSocket)→ React SPA (browser)
                                       ↕
                                 ROS2 topics (later)
```

- **nuevo_bridge** (Python FastAPI): single process that owns the serial port, decodes/encodes TLV, broadcasts to WebSocket clients, and serves the static React build
- **Frontend** (React + Vite): SPA with real-time plots (uPlot), motor controls, sensor visualization

## Project Structure

```
nuevo_ui/
├── PLAN.md                         # Original requirements/notes
├── IMPLEMENTATION_PLAN.md          # This file
├── backend/                        # Python package "nuevo_bridge"
│   ├── pyproject.toml
│   ├── tlvcodec/                   # TLV codec (moved from ros2_ws/src/tlvcodec/)
│   │   ├── __init__.py
│   │   ├── src/
│   │   │   ├── __init__.py
│   │   │   ├── encoder.py
│   │   │   ├── decoder.py
│   │   │   └── utils.py
│   │   └── tests/                  # Keep existing tests
│   ├── nuevo_bridge/
│   │   ├── __init__.py
│   │   ├── __main__.py             # Entry: uvicorn launcher
│   │   ├── app.py                  # FastAPI app, /ws endpoint, static file mount
│   │   ├── config.py               # Serial port, baud rate, heartbeat interval
│   │   ├── serial_manager.py       # Serial port owner, heartbeat, read loop
│   │   ├── ws_manager.py           # WebSocket connection manager, broadcast
│   │   ├── message_router.py       # TLV ↔ JSON translation with registry pattern
│   │   ├── payloads.py             # All ctypes structs (matching TLV_Payloads.h)
│   │   └── TLV_TypeDefs.py         # Copy of ros2_ws/src/TLV_TypeDefs.py
│   └── static/                     # Production: Vite build output
│       └── .gitkeep
├── frontend/
│   ├── package.json
│   ├── vite.config.ts
│   ├── tsconfig.json
│   ├── index.html
│   └── src/
│       ├── main.tsx
│       ├── App.tsx
│       ├── App.css
│       ├── hooks/
│       │   ├── useWebSocket.ts     # WS connection with auto-reconnect
│       │   └── useTimeSeries.ts    # Ring buffer for uPlot chart data
│       ├── store/
│       │   └── robotStore.ts       # Zustand store: all robot state
│       ├── lib/
│       │   ├── wsProtocol.ts       # TypeScript types for WS messages
│       │   └── csvExport.ts        # CSV generation + browser download
│       └── components/
│           ├── layout/
│           │   └── DashboardGrid.tsx
│           ├── system/
│           │   ├── SystemPanel.tsx
│           │   └── PowerPanel.tsx
│           ├── dc/
│           │   ├── DCMotorPanel.tsx
│           │   ├── DCMotorCard.tsx
│           │   ├── DCPIDTuner.tsx
│           │   └── DCPlot.tsx
│           ├── stepper/
│           │   ├── StepperPanel.tsx
│           │   ├── StepperCard.tsx
│           │   └── StepperPlot.tsx
│           ├── servo/
│           │   ├── ServoPanel.tsx
│           │   └── ServoChannel.tsx
│           ├── sensors/
│           │   ├── IMUPanel.tsx
│           │   ├── OrientationCube.tsx
│           │   └── RangePanel.tsx
│           ├── io/
│           │   ├── UserIOPanel.tsx
│           │   ├── ButtonGrid.tsx
│           │   ├── LEDControls.tsx
│           │   └── NeoPixelPicker.tsx
│           └── common/
│               ├── ConnectionBadge.tsx
│               ├── RecordCSV.tsx
│               └── NumberInput.tsx
└── scripts/
    ├── dev.sh                      # Start backend + frontend dev servers
    └── build.sh                    # Build frontend, copy to backend/static/
```

## Existing Code to Reuse

| File | What | How |
|------|------|-----|
| `ros2_ws/src/tlvcodec/` | TLV encoder/decoder | **Move** to `backend/tlvcodec/` |
| `ros2_ws/src/TLV_TypeDefs.py` | TLV type constants | **Copy** to `backend/nuevo_bridge/TLV_TypeDefs.py` |
| `ros2_ws/test/test_uart_arduino.py` | Working serial patterns, payload structs | Reference for `serial_manager.py` and `payloads.py` |
| `firmware/arduino/src/messages/TLV_Payloads.h` | Authoritative struct sizes | Python ctypes must match exactly |

## Backend Design

### serial_manager.py
- Runs as an asyncio background task (uses `run_in_executor` for blocking `serial.read()`)
- Sends heartbeat every 200ms (same pattern as `test_uart_arduino.py`)
- On decoded frame: calls `message_router.handle_incoming(tlv_type, tlv_data)`
- Exposes `send(tlv_type, ctypes_payload)` for outgoing commands
- Tracks stats: rx_count, tx_count, crc_errors, connected state
- Auto-reconnect on serial disconnect

### message_router.py — Registry Pattern (extensibility point)
Adding a new TLV type = **one dict entry + one ctypes struct**. No other code changes needed.

```python
# Incoming: Arduino → JSON → WebSocket broadcast
INCOMING_REGISTRY = {
    SYS_STATUS:      ("system_status",   PayloadSystemStatus),
    DC_STATUS:       ("dc_status",       PayloadDCStatus),
    SENSOR_ENCODER:  ("encoder",         PayloadSensorEncoder),
    SENSOR_VOLTAGE:  ("voltage",         PayloadSensorVoltage),
    SENSOR_CURRENT:  ("current",         PayloadSensorCurrent),
    SENSOR_IMU:      ("imu",             PayloadIMU),
    IO_BUTTON_STATE: ("buttons",         PayloadButtonState),
    IO_LIMIT_STATE:  ("limits",          PayloadLimitState),
    STEP_STATUS:     ("stepper_status",  PayloadStepStatus),
    SYS_RES_PID:     ("pid_response",    PayloadResPID),
    SENSOR_RANGE:    ("range",           PayloadSensorRange),
}

# Outgoing: JSON command → TLV → Arduino
OUTGOING_REGISTRY = {
    "dc_enable":       (DC_ENABLE,       PayloadDCEnable),
    "dc_set_velocity": (DC_SET_VELOCITY, PayloadDCSetVelocity),
    "dc_set_position": (DC_SET_POSITION, PayloadDCSetPosition),
    "set_pid":         (SYS_SET_PID,     PayloadSetPID),
    "step_enable":     (STEP_ENABLE,     PayloadStepEnable),
    "step_move":       (STEP_MOVE,       PayloadStepMove),
    "step_set_vel":    (STEP_SET_VEL,    PayloadStepSetVel),
    "step_set_accel":  (STEP_SET_ACCEL,  PayloadStepSetAccel),
    "step_home":       (STEP_HOME,       PayloadStepHome),
    "servo_enable":    (SERVO_ENABLE,    PayloadServoEnable),
    "servo_set":       (SERVO_SET,       PayloadServoSetSingle),
    "set_led":         (IO_SET_LED,      PayloadSetLED),
    "set_neopixel":    (IO_SET_NEOPIXEL, PayloadSetNeoPixel),
}
```

The router auto-converts ctypes structs to/from JSON dicts using field introspection. Array fields (e.g., `position[4]`) become JSON arrays.

### payloads.py
All ctypes structs matching `TLV_Payloads.h` exactly. Sizes verified against `STATIC_ASSERT_SIZE` values:

| Struct | Size (bytes) |
|--------|-------------|
| PayloadHeartbeat | 5 |
| PayloadSystemStatus | 18 |
| PayloadSetPID | 24 |
| PayloadGetPID | 4 |
| PayloadResPID | 24 |
| PayloadDCEnable | 4 |
| PayloadDCSetPosition | 12 |
| PayloadDCSetVelocity | 12 |
| PayloadDCStatus | 24 |
| PayloadStepEnable | 4 |
| PayloadStepSetAccel | 12 |
| PayloadStepSetVel | 8 |
| PayloadStepMove | 8 |
| PayloadStepHome | 12 |
| PayloadStepStatus | 20 |
| PayloadServoEnable | 4 |
| PayloadServoSetSingle | 4 |
| PayloadSensorVoltage | 8 |
| PayloadSensorEncoder | 36 |
| PayloadSensorCurrent | 12 |
| PayloadIMU | 24 |
| PayloadSensorRange | 8 |
| PayloadSetLED | 8 |
| PayloadSetNeoPixel | 4 |
| PayloadButtonState | 8 |
| PayloadLimitState | 8 |

### Mock Mode
Environment variable `NUEVO_MOCK=1` enables a mock serial manager that generates fake telemetry. This allows frontend development without hardware.

## WebSocket Protocol

### Server → Client (telemetry)
```json
{"topic": "encoder", "data": {"position": [1234, 0, 0, 0], "velocity": [500, 0, 0, 0], "timestamp": 12345}, "ts": 1707800000.1}
{"topic": "voltage", "data": {"batteryMv": 12400, "rail5vMv": 5020, "servoRailMv": 6000}, "ts": 1707800000.1}
{"topic": "dc_status", "data": {"motorId": 0, "enabled": true, "mode": 2, "position": 1234, "velocity": 500, "targetPos": 0, "targetVel": 500, "pwmOutput": 128, "currentMa": 350}, "ts": ...}
{"topic": "system_status", "data": {"uptimeMs": 123456, "lastHeartbeatMs": 50, "batteryMv": 12400, "rail5vMv": 5020, "servoRailMv": 6000, "errorFlags": 0, "motorEnableMask": 3, "stepperEnableMask": 0}, "ts": ...}
{"topic": "stepper_status", "data": {"stepperId": 0, "enabled": true, "state": 1, "limitHit": 0, "position": 500, "targetPos": 1000, "currentVel": 200, "stepsRemaining": 500}, "ts": ...}
{"topic": "buttons", "data": {"buttonMask": 5, "changedMask": 4, "timestamp": 123456}, "ts": ...}
{"topic": "limits", "data": {"limitMask": 2, "changedMask": 2, "timestamp": 123456}, "ts": ...}
{"topic": "imu", "data": {"accelX": 100, "accelY": -50, "accelZ": 16384, "gyroX": 0, "gyroY": 5, "gyroZ": -3, "magX": 200, "magY": 300, "magZ": -100, "temperature": 2500, "timestamp": 12345678}, "ts": ...}
{"topic": "connection", "data": {"serialConnected": true, "port": "/dev/ttyAMA0", "baud": 500000, "rxCount": 1234, "txCount": 567, "crcErrors": 2}, "ts": ...}
```

### Client → Server (commands)
```json
{"cmd": "dc_enable", "data": {"motorId": 0, "enable": 1, "mode": 2}}
{"cmd": "dc_set_velocity", "data": {"motorId": 0, "targetVel": 1000, "maxAccel": 0}}
{"cmd": "dc_set_position", "data": {"motorId": 0, "targetPos": 5000, "maxVelocity": 0}}
{"cmd": "set_pid", "data": {"motorId": 0, "motorType": 0, "loopType": 1, "kp": 0.5, "ki": 0.1, "kd": 0.0, "maxOutput": 255.0, "maxIntegral": 100.0}}
{"cmd": "step_enable", "data": {"stepperId": 0, "enable": 1}}
{"cmd": "step_move", "data": {"stepperId": 0, "moveType": 0, "target": 1000}}
{"cmd": "step_set_vel", "data": {"stepperId": 0, "maxVelocity": 500}}
{"cmd": "step_set_accel", "data": {"stepperId": 0, "accel": 1000, "decel": 1000}}
{"cmd": "step_home", "data": {"stepperId": 0, "direction": 1, "homeVelocity": 200, "backoffSteps": 50}}
{"cmd": "servo_enable", "data": {"enable": 1}}
{"cmd": "servo_set", "data": {"channel": 0, "pulseUs": 1500}}
{"cmd": "set_led", "data": {"ledId": 0, "mode": 4, "brightness": 128, "periodMs": 0, "dutyCycle": 0}}
{"cmd": "set_neopixel", "data": {"index": 0, "red": 0, "green": 255, "blue": 0}}
```

## Frontend Design
Use Frontend Design Skill for the design.

### State: Zustand store
Single store holds all robot state. WebSocket hook dispatches `topic` → store update. Components subscribe to specific slices.

### Charts: uPlot + ring buffer
- `useTimeSeries` hook: fixed-size Float64Array ring buffer (1000 points = 10s at 100Hz)
- uPlot renders at 30fps via `requestAnimationFrame` (data arrives at 100Hz but rendering is throttled)
- RecordCSV: accumulates points in separate array during recording, downloads on stop

### 3D Orientation: CSS 3D transforms
A `<div>` styled as a cube with `transform: rotateX() rotateY() rotateZ()`. No WebGL library needed.

### Layout: CSS Grid matching PCB
```
┌─────────┬────────────┬────────────┐
│  Power  │  Steppers  │   Servos   │
├─────────┼────────────┼────────────┤
│ System  │  User IO   │ DC Motors  │
└─────────┴────────────┴────────────┘
```

## Dependencies

### Backend
- `fastapi` >=0.109
- `uvicorn[standard]` >=0.27
- `pyserial` >=3.5
- `tlvcodec` (included as subpackage in `backend/tlvcodec/`)

### Frontend
- `react`, `react-dom` ^18
- `zustand` ^4.5
- `uplot` ^1.6
- Dev: `vite` ^5, `typescript` ^5, `@vitejs/plugin-react` ^4

No CSS framework — plain CSS/CSS modules.

## Development Workflow

**Dev mode** (2 terminals):
```bash
# Terminal 1: Backend (auto-reload)
cd nuevo_ui/backend && pip install -e . && uvicorn nuevo_bridge.app:app --reload --port 8000

# Terminal 2: Frontend (Vite dev server, proxies /ws and /api to backend)
cd nuevo_ui/frontend && npm install && npm run dev
```

**Production** (RPi, single process):
```bash
cd nuevo_ui && ./scripts/build.sh   # builds frontend → backend/static/
cd nuevo_ui/backend && python -m nuevo_bridge  # serves everything on port 8000
```

**Mock mode** (no Arduino):
```bash
NUEVO_MOCK=1 uvicorn nuevo_bridge.app:app --port 8000
```

## Implementation Phases

### Phase 1: Skeleton (backend + frontend connected)
- FastAPI app with `/ws` endpoint
- `serial_manager.py`: connect, heartbeat, decode callback (+ mock mode)
- `ws_manager.py`: broadcast to connected clients
- `message_router.py`: registry with `SYS_STATUS` and `SENSOR_VOLTAGE` only
- `payloads.py`: `PayloadHeartbeat`, `PayloadSystemStatus`, `PayloadSensorVoltage`
- React app: `useWebSocket` hook, `ConnectionBadge`, log messages to console
- **Verify**: Browser shows "Connected", heartbeat reaches Arduino, system_status JSON appears in console

### Phase 2: System + Power Panels
- `SystemPanel`: uptime, error flags (decoded bitmask), heartbeat status
- `PowerPanel`: battery/5V/servo voltages with colored bars
- `DashboardGrid` with CSS grid layout (only these 2 panels populated)
- Connection stats topic from backend
- **Verify**: Voltage values update live

### Phase 3: DC Motor Control
- Add `DC_STATUS`, `SENSOR_ENCODER`, `SENSOR_CURRENT` incoming + `DC_ENABLE`, `DC_SET_VELOCITY`, `DC_SET_POSITION`, `SYS_SET_PID` outgoing
- `DCMotorPanel` → `DCMotorCard` (enable, mode, setpoint inputs) + `DCPIDTuner` (Kp/Ki/Kd)
- `DCPlot` with uPlot (position, velocity, current vs time)
- `useTimeSeries` ring buffer hook
- `RecordCSV` component
- **Verify**: Enable motor, set velocity, see plots update, record CSV

### Phase 4: Stepper + Servo Control
- All stepper/servo TLV types in both registries
- `StepperPanel` → `StepperCard` (enable, velocity, accel, move target, state indicator) + `StepperPlot`
- `ServoPanel` → `ServoChannel` (slider 0-180, CSS rotate angle indicator)
- **Verify**: Move stepper, see plot. Drag servo slider, see servo respond.

### Phase 5: User IO
- `IO_BUTTON_STATE`, `IO_LIMIT_STATE`, `IO_SET_LED`, `IO_SET_NEOPIXEL` in registries
- `ButtonGrid`: 10 button indicators + 8 limit switch indicators
- `LEDControls`: toggles + brightness sliders (blue PWM, orange PWM, purple digital)
- `NeoPixelPicker`: HTML color input + brightness slider
- **Verify**: Press button, see indicator light up. Toggle LED. Pick NeoPixel color.

### Phase 6: IMU + Sensors
- `SENSOR_IMU`, `SENSOR_RANGE` in incoming registry
- `IMUPanel`: `OrientationCube` (CSS 3D), raw data table, SVG compass
- `RangePanel`: distance readout with bar
- CSV export for IMU
- **Verify**: Rotate board, cube rotates. Compass tracks heading.

### Phase 7: Polish
- Dark theme
- Error handling: serial disconnect/reconnect, WS reconnect with backoff
- `build.sh` / `dev.sh` scripts
- Layout tuning for RPi 5" display (800x480)
- REST endpoint `/api/config` for serial port selection

## Verification
1. **Backend only**: `python -m nuevo_bridge` → serial connects, heartbeat sent, Arduino responds
2. **Full stack dev**: Vite + FastAPI → browser shows live data, controls work
3. **Production build**: `build.sh` → single `uvicorn` process serves everything
4. **Mock mode**: `NUEVO_MOCK=1` → frontend shows fake data, all controls functional