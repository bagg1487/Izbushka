# app/websocket/video_manager.py
from fastapi import WebSocket
from typing import Dict

class VideoStreamManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}
    
    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections[client_id] = websocket
        print(f"Client {client_id} connected. Total: {len(self.active_connections)}")
    
    def disconnect(self, client_id: str):
        if client_id in self.active_connections:
            del self.active_connections[client_id]
            print(f"Client {client_id} disconnected. Total: {len(self.active_connections)}")
    
    async def send_frame(self, client_id: str, frame_data: dict):
        if client_id in self.active_connections:
            try:
                await self.active_connections[client_id].send_json(frame_data)
            except Exception as e:
                print(f"Send error to {client_id}: {e}")
                self.disconnect(client_id)

video_manager = VideoStreamManager()