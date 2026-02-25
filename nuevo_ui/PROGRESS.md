# NUEVO UI - Implementation Progress

**Last Updated**: 2026-02-14

## Known Issues

### ✅ DC Motor Plot Issues - FIXED (2026-02-14)

**Issue 1: Plot Not Rendering**
- **Root Cause**: Logic error in DCPlot.tsx line 39 - early return condition checked `if (!plotRef.current || timeHistory.length === 0)`, which prevented uPlot from ever being created on first render
- **Fix**: Changed line 39 to only check for empty data: `if (timeHistory.length === 0) return`
- **Result**: uPlot now initializes correctly ✅

**Issue 2: Data Compressed / Not Spreading Across X-axis**
- **Root Cause #1**: Time calculation produced descending X values `(now - t) / 1000` which gave [20, 19, ..., 0], but uPlot requires ascending X values
- **Fix #1**: Changed time calculation to `(t - oldest) / 1000` to produce ascending values [0, 0.1, ..., max]
- **Root Cause #2**: X-axis range was fixed at [0, 20], compressing data into left portion of chart when less than 20 seconds accumulated
- **Fix #2**: Changed X-axis to auto-scale: `range: (u, dataMin, dataMax) => [0, Math.max(1, dataMax * 1.05)]`
- **Updates**:
  - X-axis label changed from "Time (seconds ago)" to "Time (seconds)"
  - X-axis now dynamically scales to fit data (expands as data accumulates up to max 200 points)
  - Data fills full width of chart at all times
  - Data flows conventionally: left (oldest) → right (newest)
- **Result**: Plot auto-scales to show all data with optimal use of chart area ✅

**Issue 3: No Toggleable Legend**
- **Root Cause**: Legend markers not interactive, no click handlers
- **Fix**: Added `legend.markers.show` and `hooks.setLegend` to make legend items clickable
- **Result**: Click legend items to show/hide individual data series ✅

**Testing**: Verified with Motor 0 enabled at 500 t/s - plot shows real-time data with auto-scaling X-axis, toggleable legend, proper axes, and multi-scale support
- **Status**: FULLY RESOLVED ✅

---

## Summary

Phase 1 complete! The full skeleton is implemented:

**Backend (Python)**:
- ✅ FastAPI app with `/ws` WebSocket endpoint and `/health` REST endpoint
- ✅ Serial manager (real + mock mode) with heartbeat (200ms) and auto-reconnect
- ✅ Message router with registry pattern (extensible for new TLV types)
- ✅ WebSocket manager with broadcast to all clients
- ✅ All 26 payload ctypes structs (sizes verified against firmware)
- ✅ tlvcodec integrated (moved from ros2_ws)

**Frontend (React + TypeScript)**:
- ✅ Vite + React + TypeScript setup with WebSocket proxy
- ✅ Zustand store for robot state
- ✅ useWebSocket hook with auto-reconnect
- ✅ ConnectionBadge component (WS + Serial status)
- ✅ Basic App with debug panels showing live data

**Next**: ✅ Phases 1-3 complete! Ready for Phase 4 (Stepper + Servo Control) when needed.

---

## Phase 2: System + Power Panels

**Goal**: System panel shows uptime/error flags/heartbeat status. Power panel shows battery/5V/servo voltages with colored bars. CSS grid layout established.

### Status: ✅ COMPLETE

### Tasks:
- [x] Create DashboardGrid layout component (CSS Grid 3x2)
- [x] Implement SystemPanel with error flag decoding
- [x] Implement PowerPanel with voltage bars and colored thresholds
- [x] Update App.tsx to use grid layout with placeholders for Phase 3+
- [x] Connection stats already in backend (from Phase 1)
- [x] Style panels to match dark theme PCB layout design

### Features Delivered:
- **DashboardGrid**: 3x2 CSS grid matching PCB layout (Power/Steppers/Servos top, System/UserIO/DCMotors bottom)
- **SystemPanel**:
  - Uptime formatted (hours/minutes/seconds)
  - Heartbeat status with color coding (<100ms = green, else yellow)
  - Error flags decoded from bitmask (8 error types)
  - Motor enable masks shown as binary + hex
  - Connection stats (port, baud, RX/TX counts, CRC errors)
- **PowerPanel**:
  - Battery voltage bar (7.4-14.4V range, yellow <10.5V, orange >14V)
  - 5V rail bar (4.5-5.5V range, yellow if outside 4.9-5.1V)
  - Servo rail bar (5-10V adjustable range)
  - Animated gradient fills with color transitions

### Next: Phase 3 - DC Motor Control

---

## Phase 3: DC Motor Control

**Goal**: Full DC motor control with enable/disable, mode selection, velocity/position setpoints, PID tuning, real-time plots (position, velocity, current), and CSV export.

### Status: ✅ COMPLETE

### Tasks:
- [x] Add DC motor types to message router registries (incoming + outgoing)
- [x] Update Zustand store with DC motor state + history buffers
- [x] Create DCMotorPanel container (displays 2 motors)
- [x] Create DCMotorCard component (per-motor controls)
- [x] Create DCPIDTuner component (Kp/Ki/Kd inputs)
- [x] Create useTimeSeries hook (ring buffer for charts)
- [x] Create DCPlot component (uPlot: position, velocity, current)
- [x] Create RecordCSV component (start/stop recording, download)
- [x] Update App.tsx to use DCMotorPanel
- [x] Mock data generates sinusoidal motion at 10Hz

### Features Delivered:
- **DCMotorPanel**: Container with 2 motor cards (Motor 0 and Motor 1)
- **DCMotorCard**: Per-motor control interface
  - Enable/disable toggle with status badge
  - Mode selector (Position/Velocity/PWM)
  - Setpoint input with Set button
  - Real-time status grid (Position, Velocity, PWM, Current)
  - Embedded PID tuner (shown when enabled in position/velocity mode)
  - Live plot with 3 series (position/velocity/current)
  - CSV recording with timestamp download
- **DCPlot**: uPlot chart with optimized performance
  - Position (green), Velocity (blue), Current (orange)
  - X-axis shows relative time (seconds ago)
  - Auto-resize on window resize
- **DCPIDTuner**: PID gain configuration
  - Loop type selector (Position/Velocity)
  - Kp/Ki/Kd input fields
  - Preset defaults for each loop type
  - Apply button sends set_pid command
- **RecordCSV**: CSV export functionality
  - Start/Stop recording with animated button
  - Accumulates data during recording
  - Downloads CSV with timestamp on stop
- **Mock Data**: Realistic sinusoidal motion
  - Motor 0: sinusoidal position/velocity (1000 ticks amplitude, 1 rad/s)
  - Motor 1: linear ramp (500 ticks/sec, wraps at 5000)
  - Current varies with motion (300mA ± 50mA)
  - 10Hz update rate for smooth animations

### WebSocket Commands Implemented:
- `dc_enable`: Enable/disable motor with mode selection
- `dc_set_velocity`: Set target velocity with max acceleration
- `dc_set_position`: Set target position with max velocity
- `set_pid`: Configure PID gains for position/velocity loops

### Next: Phase 4 - Stepper + Servo Control

---

## Phase 1: Skeleton (backend + frontend connected)

**Goal**: FastAPI serves a React page, WebSocket connection established, heartbeat sent to Arduino, raw TLV data displayed in browser console.

### Status: ✅ COMPLETE (code ready, needs testing with `npm install` + browser)

### Tasks:
- [x] Move tlvcodec from ros2_ws/src/ to backend/
- [x] Copy TLV_TypeDefs.py to backend/nuevo_bridge/
- [x] Create backend structure with pyproject.toml
- [x] Implement config.py
- [x] Implement payloads.py (all 26 payloads, sizes verified)
- [x] Implement ws_manager.py
- [x] Implement message_router.py (minimal registry for Phase 1)
- [x] Implement serial_manager.py (with mock mode)
- [x] Implement app.py (FastAPI + WebSocket)
- [x] Implement __main__.py
- [x] Create frontend structure with Vite + React + TypeScript
- [x] Implement useWebSocket hook
- [x] Implement robotStore (Zustand)
- [x] Implement ConnectionBadge component
- [x] Implement basic App.tsx

### Testing:
- [x] Backend imports successfully
- [ ] Backend runs in mock mode
- [ ] Frontend builds and runs
- [ ] Full stack: browser connects via WebSocket
- [ ] Console shows incoming messages (system_status, voltage, connection)
- [ ] Connection badge shows green when connected

---

## Testing Instructions

### Backend Only (Mock Mode)
```bash
cd nuevo_ui/backend
NUEVO_MOCK=1 python3 -m nuevo_bridge
# Should see:
# [App] Starting NUEVO Bridge (mock=True)...
# [Serial] Starting MOCK serial manager...
# INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Frontend Only
```bash
cd nuevo_ui/frontend
npm install  # First time only
npm run dev
# Should see:
# VITE v5.x.x  ready in xxx ms
# ➜  Local:   http://localhost:5173/
```

### Full Stack (Recommended)
```bash
cd nuevo_ui
./scripts/dev.sh
# Opens frontend at http://localhost:5173
# Backend runs at http://localhost:8000
```

### What to Verify:
1. **Backend running**: Visit http://localhost:8000/health - should return JSON with `"status": "ok"`
2. **Frontend loads**: Visit http://localhost:5173 - should see "NUEVO Dashboard" header
3. **WebSocket connects**: Connection badge shows green "WS: Connected"
4. **Serial status**: Badge shows "Serial: Connected" (mock mode always connected)
5. **Console messages**: Open browser devtools → Console → see `[WS] system_status {...}` and `[WS] voltage {...}` every 1 second
6. **Data displays**: System Status and Voltage panels show JSON data updating live

---

## Test Results

### Backend Tests
✓ All 26 payload sizes verified (matches firmware STATIC_ASSERT_SIZE)
✓ Backend imports successfully
✓ Mock serial manager generates fake data

### Frontend Tests
(Pending npm install + browser test)

### Integration Tests
(Pending full stack test)
