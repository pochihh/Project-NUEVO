# NUEVO UI

Web-based monitoring and control interface for the MAE 162 robot platform.
A React/TypeScript frontend communicates over WebSocket with a FastAPI backend
that bridges to the Arduino via UART serial.

```
Browser  ←──WebSocket──→  FastAPI backend  ←──UART──→  Arduino
(React)                   (nuevo_bridge)               (firmware)
                               │
                         ROS2 topics (optional, NUEVO_ROS2=1)
```

---

## Directory structure

```
nuevo_ui/
├── frontend/          React + TypeScript + Vite source
│   ├── src/
│   │   ├── app/       App layout and page components
│   │   ├── hooks/     useWebSocket (connection + auto-reconnect)
│   │   ├── lib/       wsProtocol.ts, wsSend.ts
│   │   ├── store/     Zustand state (robotStore, authStore)
│   │   └── styles/    Global CSS (Tailwind v4)
│   └── dist/          Vite build output → copy to backend/static/
├── backend/
│   ├── nuevo_bridge/  FastAPI app, serial manager, message router
│   ├── static/        Compiled frontend (served by FastAPI)
│   └── tlvcodec/      TLV encoder/decoder (matches Arduino firmware)
└── scripts/
    └── dev.sh         Start backend + frontend in parallel (development)
```

---

## Development mode

Use the provided script to start both servers simultaneously:

```bash
cd nuevo_ui
./scripts/dev.sh
```

This starts:
- **Backend** at `http://localhost:8000` — mock mode (no Arduino needed)
- **Frontend** at `http://localhost:5173` — Vite dev server with hot reload

Open `http://localhost:5173` in your browser. The frontend proxies `/ws` and `/auth`
requests to the backend automatically (configured in `vite.config.ts`).

### Manual startup (two terminals)

**Terminal 1 — backend (mock mode)**
```bash
cd nuevo_ui/backend
NUEVO_MOCK=1 python3 -m nuevo_bridge
```

**Terminal 2 — frontend dev server**
```bash
cd nuevo_ui/frontend
npm install    # first time only
npm run dev
```

---

## Production build

The frontend must be compiled to static files so the FastAPI backend can serve it
as a single application on one port. This is required when running on the RPi or
inside Docker.

### 1. Install dependencies (first time only)

```bash
cd nuevo_ui/frontend
npm install
```

### 2. Build

```bash
cd nuevo_ui/frontend
npm run build
```

Vite compiles the app and writes output to `frontend/dist/`.

### 3. Copy to backend static directory

```bash
# From nuevo_ui/
cp -r frontend/dist/. backend/static/
```

After this the backend serves the full UI at `http://<device>:8000`.

### One-liner (from `nuevo_ui/`)

```bash
cd frontend && npm run build && cd .. && cp -r frontend/dist/. backend/static/
```

---

## Running the backend

### Plain Python (no ROS2, mock mode)

```bash
cd nuevo_ui/backend
NUEVO_MOCK=1 python3 -m nuevo_bridge
```

### Plain Python (real Arduino)

```bash
cd nuevo_ui/backend
NUEVO_SERIAL_PORT=/dev/ttyAMA0 python3 -m nuevo_bridge
```

### With ROS2 (inside Docker or on Ubuntu 24.04 with ROS2 installed)

See [`../ros2_ws/README.md`](../ros2_ws/README.md) for Docker setup. The bridge is started
automatically by the Docker entrypoint with `NUEVO_ROS2=1`.

---

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `NUEVO_MOCK` | `0` | `1` = run without Arduino (simulated data) |
| `NUEVO_ROS2` | `0` | `1` = enable ROS2 topic publishing/subscribing |
| `NUEVO_SERIAL_PORT` | `/dev/ttyAMA0` | Serial device path |
| `NUEVO_SERIAL_BAUD` | `1000000` | Baud rate (must match firmware) |

---

## WebSocket API

The frontend connects to `ws://<host>:8000/ws?token=<jwt>`. All messages are JSON.

**Bridge → browser (topics):**

| Topic | Description |
|-------|-------------|
| `system_status` | Arduino state, firmware version, loop timing, errors |
| `voltage` | Battery mV, 5V rail mV, servo rail mV |
| `dc_status_all` | Position, velocity, PWM, PID state for all 4 DC motors |
| `step_status_all` | State, position, speed for all 4 steppers |
| `servo_status_all` | Enable mask and pulse width for all 16 servo channels |
| `io_status` | Button states, LED brightness, NeoPixel colors |
| `kinematics` | x, y, θ, vx, vy, ωz from wheel odometry |
| `imu` | Quaternion, raw accel/gyro, magnetometer, calibration state |
| `connection` | Serial port stats (rx/tx bytes, CRC errors) |

**Browser → bridge (commands):** `dc_enable`, `dc_set_velocity`, `dc_set_pwm`,
`dc_set_position`, `set_pid`, `step_enable`, `step_move`, `step_home`,
`step_set_params`, `servo_enable`, `servo_set`, `set_led`, `set_neopixel`,
`sys_cmd`, `mag_cal_cmd`.

See `backend/nuevo_bridge/message_router.py` for full field definitions.
