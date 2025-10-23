# main.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
import json
import uvicorn
import threading

app = FastAPI()

current_image = "/static/default.png"

class ConnectionManager:
    def __init__(self):
        self.active_connections = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        await self.send_current_image(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def send_current_image(self, websocket: WebSocket):
        await websocket.send_text(json.dumps({"type": "image_update", "image": current_image}))

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                self.disconnect(connection)

manager = ConnectionManager()

app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/")
async def read_index():
    return FileResponse("templates/index.html")

@app.get("/casino")
async def read_casino():
    return FileResponse("templates/casino.html")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

class ImageChange(BaseModel):
    image: str

@app.post("/change-image")
async def change_image(data: ImageChange):
    global current_image
    current_image = data.image
    await manager.broadcast(json.dumps({"type": "image_update", "image": current_image}))
    return {"status": "success"}

@app.post("/casino/spin")
async def casino_spin():
    await manager.broadcast(json.dumps({"type": "casino_spin"}))
    return {"status": "success"}

def start_voice_processor():
    from audio import VoiceProcessor
    voice_processor = VoiceProcessor()
    voice_processor.listen()

if __name__ == "__main__":
    voice_thread = threading.Thread(target=start_voice_processor)
    voice_thread.daemon = True
    voice_thread.start()
    uvicorn.run(app, host="127.0.0.1", port=8000)