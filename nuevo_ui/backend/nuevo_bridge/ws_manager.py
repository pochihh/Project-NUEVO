"""
WebSocket Connection Manager

Manages WebSocket clients and broadcasts messages to all connected clients.
"""
import json
import asyncio
from typing import Set
from fastapi import WebSocket


class WSManager:
    """Manages WebSocket connections and broadcasting"""

    def __init__(self):
        self.connections: Set[WebSocket] = set()

    async def connect(self, websocket: WebSocket):
        """Accept a new WebSocket connection"""
        await websocket.accept()
        self.connections.add(websocket)
        print(f"[WS] Client connected. Total connections: {len(self.connections)}")

    def disconnect(self, websocket: WebSocket):
        """Remove a WebSocket connection"""
        self.connections.discard(websocket)
        print(f"[WS] Client disconnected. Total connections: {len(self.connections)}")

    async def broadcast(self, message: dict):
        """Broadcast a JSON message to all connected clients"""
        if not self.connections:
            return

        # Serialize once
        json_str = json.dumps(message)

        # Track dead connections
        dead_connections = set()

        for websocket in self.connections:
            try:
                await websocket.send_text(json_str)
            except Exception as e:
                print(f"[WS] Error sending to client: {e}")
                dead_connections.add(websocket)

        # Clean up dead connections
        for websocket in dead_connections:
            self.disconnect(websocket)

    async def send_to(self, websocket: WebSocket, message: dict):
        """Send a JSON message to a specific client"""
        try:
            await websocket.send_json(message)
        except Exception as e:
            print(f"[WS] Error sending to specific client: {e}")
            self.disconnect(websocket)

    def get_connection_count(self) -> int:
        """Get the number of active connections"""
        return len(self.connections)


# Global instance
ws_manager = WSManager()
