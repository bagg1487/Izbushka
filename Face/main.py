import webbrowser
import asyncio
import threading
import time
import queue
import cv2
import numpy as np
import json

from fastapi.responses import FileResponse, StreamingResponse
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware

from pydantic import BaseModel
import uvicorn

from reaction import get_random_expression

app = FastAPI()

# Добавляем CORS middleware для работы с WebSocket
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Глобальные переменные для видеопотока
frame_queue = queue.Queue(maxsize=2)
latest_frame = None
frame_lock = threading.Lock()
streaming_active = False
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


@app.on_event("startup")
async def startup_event():
    """Запуск потока захвата видео при старте приложения"""
    global streaming_active
    streaming_active = True
    capture_thread = threading.Thread(target=video_capture_worker, daemon=True)
    capture_thread.start()
    print("Запущен поток захвата видео")


@app.on_event("shutdown")
async def shutdown_event():
    """Остановка потока захвата видео при завершении приложения"""
    global streaming_active
    streaming_active = False
    print("Остановлен поток захвата видео")


def video_capture_worker():
    """Рабочий поток для захвата видео с камеры"""
    global latest_frame, streaming_active

    # Пробуем разные индексы камер
    cap = None
    for camera_index in range(3):
        try:
            cap = cv2.VideoCapture(camera_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)

            if cap.isOpened():
                print(f"Камера найдена на индексе {camera_index}")
                break
        except:
            if cap:
                cap.release()

    if cap is None or not cap.isOpened():
        print("Не удалось открыть камеру")
        # Создаем тестовое изображение для демонстрации
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(test_frame, "КАМЕРА НЕ НАЙДЕНА", (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        while streaming_active:
            with frame_lock:
                latest_frame = test_frame.copy()
            time.sleep(0.033)  # ~30 FPS
        return

    while streaming_active:
        try:
            ret, frame = cap.read()
            if ret:
                frame = cv2.flip(frame, 1)  # Зеркальное отражение

                with frame_lock:
                    latest_frame = frame

                # Добавляем кадр в очередь для режима рисования
                if not frame_queue.full():
                    try:
                        frame_queue.put(frame.copy(), timeout=0.01)
                    except queue.Full:
                        pass
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Ошибка захвата видео: {e}")
            time.sleep(0.1)

    cap.release()
    print("Поток захвата видео завершен")


def get_latest_frame():
    """Получение последнего кадра с блокировкой"""
    with frame_lock:
        if latest_frame is not None:
            return latest_frame.copy()
    return None


def update_frame_for_streaming(frame):
    """Обновление кадра для трансляции"""
    with frame_lock:
        global latest_frame
        latest_frame = frame


@app.websocket("/ws/diagnostics")
async def diagnostics_websocket(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
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
            await asyncio.sleep(1)

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

@app.get("/drawing")
async def read_drawing():
    return FileResponse("templates/drawing.html")

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

@app.post("/drawing/start")
async def start_drawing():
    """Запуск режима рисования"""
    try:
        import camera
        success = camera.start_drawing_mode(frame_queue, update_frame_for_streaming)
        if success:
            await manager.broadcast(json.dumps({
                "type": "drawing_status",
                "active": True
            }))
            return {"status": "success", "message": "Рисование запущено"}
        return {"status": "error", "message": "Рисование уже активно"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/drawing/stop")
async def stop_drawing():
    """Остановка режима рисования"""
    try:
        import camera
        success = camera.stop_drawing_mode()
        if success:
            await manager.broadcast(json.dumps({
                "type": "drawing_status",
                "active": False
            }))
            return {"status": "success", "message": "Рисование остановлено"}
        return {"status": "error", "message": "Рисование не было активно"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.get("/drawing/status")
async def get_drawing_status():
    """Получение статуса рисования"""
    try:
        import camera
        return camera.get_drawing_status()
    except:
        return {"active": False}


@app.get("/video_feed")
async def video_feed():
    """Потоковое видео с камеры для веб-интерфейса"""

    async def generate_frames():
        while True:
            frame = get_latest_frame()
            if frame is not None:
                # Конвертируем BGR в RGB для веб-отображения
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                ret, buffer = cv2.imencode('.jpg', frame_rgb, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           buffer.tobytes() + b'\r\n')
            else:
                # Если кадра нет, создаем черный кадр
                blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                ret, buffer = cv2.imencode('.jpg', blank_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           buffer.tobytes() + b'\r\n')

            await asyncio.sleep(0.033)  # ~30 FPS

    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

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

    elif command == "рисуй":
        response = await start_drawing()
        if response.get("status") == "success":
            await manager.broadcast(json.dumps({
                "type": "notification",
                "message": "🎨 Режим рисования запущен!"
            }))
        return response

    elif command == "стоп рисование":
        response = await stop_drawing()
        if response.get("status") == "success":
            await manager.broadcast(json.dumps({
                "type": "notification",
                "message": "🛑 Рисование остановлено"
            }))
        return response

    return {"status": "error", "message": "Неизвестная команда"}

def start_voice_processor():
    from audio import VoiceProcessor
    vp = VoiceProcessor()
    vp.listen()
    while True:
        time.sleep(1)


if __name__ == "__main__":

    voice_thread = threading.Thread(target=start_voice_processor, daemon=True)
    voice_thread.start()
    uvicorn.run(app, host="127.0.0.1", port=8000)