from fastapi import WebSocket
from typing import Dict

class VideoStreamManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}
    
    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections[client_id] = websocket
    
    def disconnect(self, client_id: str):
        if client_id in self.active_connections:
            del self.active_connections[client_id]
    
    async def send_frame(self, client_id: str, frame_data: dict):
        if client_id in self.active_connections:
            try:
                await self.active_connections[client_id].send_json(frame_data)
            except:
                self.disconnect(client_id)

video_manager = VideoStreamManager()