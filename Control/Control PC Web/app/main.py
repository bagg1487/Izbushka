from fastapi import FastAPI, Depends, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi import WebSocket, WebSocketDisconnect
import socket
import cv2
import asyncio
import base64
import threading
import os
from contextlib import asynccontextmanager
import logging
from dotenv import load_dotenv

from app.api.calibration import router as calibration_router
from app.api.commands import router as commands_router
from app.api.users import router as users_router
from app.api.auth import router as auth_router
from app.auth import get_current_user

load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ROBOT_IP = os.getenv("ROBOT_IP", "192.168.15.2")
ROBOT_COMMAND_PORT = int(os.getenv("ROBOT_COMMAND_PORT", "5001"))
RTSP_URL = os.getenv("RTSP_URL", "rtsp://192.168.15.2:8554/stream")

robot_socket = None
robot_connected = False
rtsp_cap = None
rtsp_running = False
active_websockets = []
last_frame = None
frame_lock = threading.Lock()

def send_command(cmd: str):
    global robot_socket, robot_connected
    if robot_connected and robot_socket:
        try:
            command = cmd + '\n'
            robot_socket.send(command.encode('utf-8'))
            return True
        except Exception as e:
            logger.error(f"Command send error: {e}")
            robot_connected = False
    return False

def rtsp_capture_thread():
    global last_frame, rtsp_running, rtsp_cap, RTSP_URL
    while rtsp_running:
        try:
            if rtsp_cap is None or not rtsp_cap.isOpened():
                rtsp_cap = cv2.VideoCapture(RTSP_URL)
                continue
            ret, frame = rtsp_cap.read()
            if ret and frame is not None:
                with frame_lock:
                    last_frame = frame
        except Exception as e:
            logger.error(f"RTSP capture error: {e}")
        cv2.waitKey(1)

@asynccontextmanager
async def lifespan(app: FastAPI):
    global robot_socket, robot_connected, rtsp_cap, rtsp_running
    
    print("Server started")
    os.makedirs("app/static/img", exist_ok=True)
    
    try:
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.settimeout(5)
        robot_socket.connect((ROBOT_IP, ROBOT_COMMAND_PORT))
        robot_connected = True
        logger.info(f"Connected to robot at {ROBOT_IP}:{ROBOT_COMMAND_PORT}")
    except Exception as e:
        logger.error(f"Failed to connect to robot: {e}")
        robot_connected = False
    
    rtsp_cap = cv2.VideoCapture(RTSP_URL)
    if rtsp_cap.isOpened():
        rtsp_running = True
        logger.info(f"RTSP stream opened: {RTSP_URL}")
        thread = threading.Thread(target=rtsp_capture_thread, daemon=True)
        thread.start()
    else:
        logger.error(f"Failed to open RTSP stream: {RTSP_URL}")
    
    yield
    
    rtsp_running = False
    if robot_socket:
        robot_socket.close()
    if rtsp_cap:
        rtsp_cap.release()
    print("Server stopped")

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth_router)
app.include_router(calibration_router)
app.include_router(commands_router)
app.include_router(users_router)

app.mount("/static", StaticFiles(directory="app/static"), name="static")

@app.get("/")
async def serve_index():
    index_path = os.path.join("app", "static", "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"error": "index.html not found"}

@app.get("/control")
async def serve_control(current_user = Depends(get_current_user)):
    control_path = os.path.join("app", "static", "control.html")
    return FileResponse(control_path)

@app.get("/video")
async def serve_video(current_user = Depends(get_current_user)):
    video_path = os.path.join("app", "static", "video.html")
    return FileResponse(video_path)

@app.get("/info")
async def serve_info(current_user = Depends(get_current_user)):
    info_path = os.path.join("app", "static", "info.html")
    return FileResponse(info_path)

@app.get("/status")
async def status(current_user = Depends(get_current_user)):
    return {
        "connected": robot_connected,
        "robot_ip": ROBOT_IP if robot_connected else None,
        "rtsp_url": RTSP_URL
    }

@app.websocket("/ws/video")
async def video_stream(websocket: WebSocket):
    global last_frame, active_websockets
    await websocket.accept()
    active_websockets.append(websocket)
    try:
        while True:
            if last_frame is not None:
                with frame_lock:
                    frame = last_frame.copy()
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                frame_base64 = base64.b64encode(buffer).decode('utf-8')
                await websocket.send_json({"type": "video", "data": frame_base64})
            await asyncio.sleep(1/30)
    except WebSocketDisconnect:
        active_websockets.remove(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if websocket in active_websockets:
            active_websockets.remove(websocket)