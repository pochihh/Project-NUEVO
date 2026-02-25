"""
NUEVO Bridge - FastAPI Application

Main application entry point:
- WebSocket endpoint at /ws
- Serves static React build in production
- Manages serial manager lifecycle
"""
import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import os

from .config import MOCK_MODE
from .ws_manager import ws_manager
from .message_router import MessageRouter
from .serial_manager import SerialManager, MockSerialManager


# Global instances
message_router = MessageRouter(ws_manager)

if MOCK_MODE:
    serial_manager = MockSerialManager(message_router, ws_manager)
else:
    serial_manager = SerialManager(message_router, ws_manager)

serial_task = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage serial manager lifecycle"""
    global serial_task

    # Startup
    print(f"[App] Starting NUEVO Bridge (mock={MOCK_MODE})...")
    serial_task = asyncio.create_task(serial_manager.run())

    yield

    # Shutdown
    print("[App] Shutting down...")
    serial_manager.stop()
    if serial_task:
        await serial_task


app = FastAPI(title="NUEVO Bridge", lifespan=lifespan)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for frontend communication"""
    await ws_manager.connect(websocket)

    try:
        while True:
            # Receive commands from frontend
            data = await websocket.receive_json()

            # Route command to Arduino (Phase 3+)
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


# Serve static files in production (after frontend build)
static_dir = os.path.join(os.path.dirname(__file__), "..", "static")
if os.path.exists(static_dir) and os.listdir(static_dir):
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

    @app.get("/")
    async def serve_index():
        return FileResponse(os.path.join(static_dir, "index.html"))


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "ok",
        "mock_mode": MOCK_MODE,
        "serial_connected": serial_manager.stats.get("connected", False),
        "ws_connections": ws_manager.get_connection_count(),
    }
