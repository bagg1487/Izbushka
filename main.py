from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
import json
import uvicorn
import threading

app = FastAPI()

current_expression = "neutral"

class ConnectionManager:
    def __init__(self):
        self.active_connections = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        await self.send_current_expression(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def send_current_expression(self, websocket: WebSocket):
        await websocket.send_text(json.dumps({
            "type": "face_expression", 
            "expression": current_expression
        }))

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
    return FileResponse("templates/face.html")

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

class FaceExpression(BaseModel):
    expression: str

@app.post("/change-expression")
async def change_expression(data: FaceExpression):
    global current_expression
    current_expression = data.expression
    await manager.broadcast(json.dumps({
        "type": "face_expression", 
        "expression": current_expression
    }))
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