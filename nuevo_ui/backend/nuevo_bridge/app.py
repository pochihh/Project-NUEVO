"""
NUEVO Bridge - FastAPI Application

Main application entry point:
- Auth REST API at /auth/* (login, user management)
- WebSocket endpoint at /ws?token=<jwt> (requires valid token)
- Serves static React build in production
- Manages serial manager lifecycle

ROS2 mode (NUEVO_ROS2=1):
  - Creates NuevoBridgeNode that publishes all telemetry to ROS2 topics
  - Subscribes to /nuevo/cmd_vel, /nuevo/step_cmd, etc. for incoming commands
  - rclpy.spin() runs in a dedicated daemon thread
  - The WebSocket UI remains fully functional alongside ROS2
"""
import asyncio
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from .config import MOCK_MODE, ROS2_MODE
from .ws_manager import ws_manager
from .message_router import MessageRouter
from .serial_manager import SerialManager, MockSerialManager
from .auth import decode_token
from .auth_router import router as auth_router

# ── Optional ROS2 import (only when NUEVO_ROS2=1) ────────────────────────────
_ros2_available = False
if ROS2_MODE:
    try:
        import rclpy
        from .ros2_node import NuevoBridgeNode
        _ros2_available = True
    except ImportError:
        print("[App] WARNING: NUEVO_ROS2=1 but rclpy not installed — "
              "running in pure Python mode")

# ── Global instances ──────────────────────────────────────────────────────────
message_router = MessageRouter(ws_manager)

if MOCK_MODE:
    serial_manager = MockSerialManager(message_router, ws_manager)
else:
    serial_manager = SerialManager(message_router, ws_manager)

serial_task  = None
ros2_node    = None
ros2_thread  = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global serial_task, ros2_node, ros2_thread

    print(f"[App] Starting NUEVO Bridge (mock={MOCK_MODE}, ros2={_ros2_available})...")

    # ── ROS2 startup ─────────────────────────────────────────────────────────
    if _ros2_available:
        rclpy.init()
        ros2_node = NuevoBridgeNode(serial_manager, message_router)
        serial_manager.set_ros2_node(ros2_node)
        ros2_thread = ros2_node.spin_in_thread()
        print("[App] ROS2 node started — publishing to /odom, /imu/*, /nuevo/*")

    # ── Serial manager startup ────────────────────────────────────────────────
    serial_task = asyncio.create_task(serial_manager.run())

    yield  # ── Application runs here ─────────────────────────────────────────

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print("[App] Shutting down...")
    serial_manager.stop()
    if serial_task:
        await serial_task

    if _ros2_available and ros2_node is not None:
        ros2_node.destroy_node()
        rclpy.shutdown()
        # ros2_thread is daemon — it will exit when rclpy.spin() returns


app = FastAPI(title="NUEVO Bridge", lifespan=lifespan)

# Auth routes — must be registered before the static file catch-all mount
app.include_router(auth_router)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, token: str = Query(None)):
    """WebSocket endpoint. Requires a valid JWT token as ?token= query param."""
    # Validate token before accepting the connection
    try:
        if not token:
            raise ValueError("No token provided")
        decode_token(token)  # raises HTTPException on invalid/expired token
    except Exception:
        await websocket.close(code=4001)
        return

    await ws_manager.connect(websocket)

    try:
        while True:
            data = await websocket.receive_json()

            cmd = data.get("cmd")
            cmd_data = data.get("data", {})

            result = message_router.handle_outgoing(cmd, cmd_data)
            if result:
                tlv_type, payload = result
                serial_manager.send(tlv_type, payload)

    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
    except Exception as e:
        print(f"[WS] Error in websocket handler: {e}")
        ws_manager.disconnect(websocket)


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "ok",
        "mock_mode": MOCK_MODE,
        "ros2_mode": _ros2_available,
        "serial_connected": serial_manager.stats.get("connected", False),
        "ws_connections": ws_manager.get_connection_count(),
    }


# Serve static files in production (after frontend build).
# Mounted last so explicit routes above take precedence.
static_dir = os.path.join(os.path.dirname(__file__), "..", "static")
if os.path.exists(static_dir) and os.listdir(static_dir):
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

    @app.get("/")
    async def serve_index():
        return FileResponse(os.path.join(static_dir, "index.html"))
