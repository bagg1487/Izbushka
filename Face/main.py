import webbrowser
import asyncio
from reaction import get_random_expression

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
import json
import uvicorn
import threading

app = FastAPI()
# hello
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

@app.websocket("/ws/diagnostics")
async def diagnostics_websocket(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Генерируем тестовые данные датчиков
            import random
            from datetime import datetime
            
            sensor_data = {
                "timestamp": datetime.now().isoformat(),
                "distance": max(0, 50 + random.uniform(-10, 10)),
                "gyro_x": random.uniform(-30, 30),
                "gyro_y": random.uniform(-30, 30),
                "gyro_z": random.uniform(-10, 10)
            }
            
            await websocket.send_text(json.dumps({
                "type": "sensor_data",
                "data": sensor_data
            }))
            await asyncio.sleep(1)  # Обновление каждую секунду
            
    except Exception as e:
        print(f"WebSocket error: {e}")

@app.get("/")
async def read_index():
    return FileResponse("templates/face.html")

@app.get("/casino")
async def read_casino():
    return FileResponse("templates/casino.html")

@app.get("/control")
async def read_control():
    return FileResponse("templates/control.html")

@app.get("/diagnostics")
async def read_diagnostics():
    return FileResponse("templates/diagnostics.html")

@app.get("/current-expression")
async def get_current_expression():
    return {"expression": current_expression}

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

class TextCommand(BaseModel):
    command: str

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

@app.post("/text-command")
async def text_command(data: TextCommand):
    global current_expression
    command = data.command.lower()
    
    expressions = {
        "реакция": "random",
        "радость": "happy", 
        "злость": "angry",
        "удивление": "surprised",
        "грусть": "sad",
        "смех": "laughing",
        "подмигни": "winking",
        "сон": "sleepy",
        "нейтрально": "neutral"
    }
    
    if command in expressions:
        if command == "реакция":
            expression = get_random_expression()
        else:
            expression = expressions[command]
        
        current_expression = expression
        await manager.broadcast(json.dumps({
            "type": "face_expression", 
            "expression": current_expression
        }))
        return {"status": "success", "expression": expression}
    
    elif command == "крути":
        await manager.broadcast(json.dumps({"type": "casino_spin"}))
        return {"status": "success", "action": "casino_spin"}
    
    elif command == "диагностика":
        webbrowser.open("http://127.0.0.1:8000/diagnostics")
        return {"status": "success", "action": "open_diagnostics"}
    
    return {"status": "error", "message": "Неизвестная команда"}

def start_voice_processor():
    from audio import VoiceProcessor
    vp = VoiceProcessor()
    vp.listen()

if __name__ == "__main__":
    voice_thread = threading.Thread(target=start_voice_processor)
    voice_thread.daemon = True
    voice_thread.start()

    uvicorn.run(app, host="127.0.0.1", port=8000)
