from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
import socket
import struct
import cv2
import numpy as np
import asyncio
import base64
import threading
import subprocess
import platform
import os
from contextlib import asynccontextmanager

from app.api import calibration, commands, users
from app.websocket import video_manager

robot_socket = None
robot_video_socket = None
robot_connected = False
current_ip = None
video_thread_running = False
last_frame = None
sonic_value = "0"

frame_lock = threading.Lock()
sonic_lock = threading.Lock()

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Server started")
    os.makedirs("app/static/img", exist_ok=True)
    yield
    global robot_socket, robot_video_socket, robot_connected
    if robot_socket:
        robot_socket.close()
    if robot_video_socket:
        robot_video_socket.close()
    robot_connected = False
    print("Server stopped")

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(calibration.router)
app.include_router(commands.router)
app.include_router(users.router)

app.mount("/static", StaticFiles(directory="app/static"), name="static")

@app.get("/")
async def serve_index():
    index_path = os.path.join("app", "static", "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"error": "index.html not found"}

@app.get("/video")
async def serve_video_page():
    video_path = os.path.join("app", "static", "video.html")
    if os.path.exists(video_path):
        return FileResponse(video_path)
    return {"error": "video.html not found"}

@app.post("/connect")
async def connect(ip: dict):  # <-- принимаем как словарь
    global robot_socket, robot_video_socket, robot_connected, current_ip, video_thread_running
    
    try:
        ip_address = ip.get("ip")  # <-- достаем ip из словаря
        if not ip_address:
            raise HTTPException(400, "IP address required")
            
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.connect((ip_address, 5001))
        
        robot_video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_video_socket.connect((ip_address, 8001))
        
        robot_connected = True
        current_ip = ip_address
        
        if not video_thread_running:
            video_thread = threading.Thread(target=video_receiver, daemon=True)
            video_thread.start()
            video_thread_running = True
        
        with open('IP.txt', 'w') as f:
            f.write(ip_address)
            
        return {"status": "connected", "ip": ip_address}
    except Exception as e:
        raise HTTPException(500, f"Connection failed: {e}")

@app.post("/disconnect")
async def disconnect():
    global robot_socket, robot_video_socket, robot_connected
    robot_connected = False
    if robot_socket:
        robot_socket.close()
    if robot_video_socket:
        robot_video_socket.close()
    return {"status": "disconnected"}

@app.get("/status")
async def status():
    return {
        "connected": robot_connected,
        "ip": current_ip if robot_connected else None
    }

@app.get("/api/ssh")
async def get_ssh_info():
    system = platform.system()
    try:
        if system == "Windows":
            result = subprocess.run(['ipconfig'], capture_output=True, text=True)
            import re
            ip_match = re.search(r'IPv4-адрес[^\d]*([\d.]+)', result.stdout)
            local_ip = ip_match.group(1) if ip_match else 'unknown'
        else:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            local_ip = result.stdout.strip().split()[0] if result.stdout else 'unknown'
        
        username = subprocess.run(['whoami'], capture_output=True, text=True).stdout.strip()
        
        return {
            "status": "ok",
            "ssh_command": f"ssh {username}@{local_ip} -p 22",
            "username": username,
            "ip": local_ip,
            "port": 22
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/api/anydesk")
async def get_anydesk_info():
    try:
        system = platform.system()
        
        if system == "Windows":
            possible_paths = [
                r"C:\Program Files (x86)\AnyDesk\AnyDesk.exe",
                r"C:\Program Files\AnyDesk\AnyDesk.exe"
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    result = subprocess.run([path, '--get-id'], capture_output=True, text=True)
                    anydesk_id = result.stdout.strip()
                    return {
                        "status": "ok",
                        "anydesk_id": anydesk_id,
                        "message": f"AnyDesk ID: {anydesk_id}"
                    }
        else:
            result = subprocess.run(['anydesk', '--get-id'], capture_output=True, text=True)
            anydesk_id = result.stdout.strip()
            return {
                "status": "ok",
                "anydesk_id": anydesk_id,
                "message": f"AnyDesk ID: {anydesk_id}"
            }
        
        return {"status": "error", "message": "AnyDesk not found"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def video_receiver():
    global last_frame, robot_video_socket, robot_connected

    
    while robot_connected:
        try:
            
            stream_bytes = robot_video_socket.recv(4)
          
            if not stream_bytes or len(stream_bytes) < 4:
                print("⚠️ No video data received, continuing...")
                continue
                
            leng = struct.unpack('<L', stream_bytes[:4])[0]
          
            
            # Читаем сам кадр
            jpg = b''
            bytes_received = 0
            while bytes_received < leng:
                chunk = robot_video_socket.recv(min(4096, leng - bytes_received))
                if not chunk:
                    print("⚠️ Connection closed while reading frame")
                    break
                jpg += chunk
                bytes_received += len(chunk)
          
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                with frame_lock:
                    last_frame = frame
                
            else:
                print("❌ Failed to decode frame")
                
        except Exception as e:
            print(f"🔥 Video receiver error: {e}")
            import traceback
            traceback.print_exc()
            break
    
    print("💀 Video receiver thread stopped")

def send_command(cmd: str):
    global robot_socket, robot_connected
    if robot_connected and robot_socket:
        try:
            command = cmd + '\n'
            robot_socket.send(command.encode('utf-8'))
            print(f"📤 Sending: {command.strip()}")
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            robot_connected = False
    return False

@app.get("/control")
async def serve_control():
    control_path = os.path.join("app", "static", "control.html")
    if os.path.exists(control_path):
        return FileResponse(control_path)
    return {"error": "control.html not found"}    

@app.websocket("/ws/video")
async def video_stream(websocket: WebSocket):
    client_id = str(id(websocket))
    await video_manager.connect(websocket, client_id)
    try:
        while True:
            if robot_connected and last_frame is not None:
                with frame_lock:
                    frame = last_frame.copy()
                
                _, buffer = cv2.imencode('.jpg', frame)
                frame_base64 = base64.b64encode(buffer).decode('utf-8')
                
                await video_manager.send_frame(client_id, {
                    "type": "video",
                    "data": frame_base64,
                    "sonic": sonic_value
                })
            
            await asyncio.sleep(0.03)
    except:
        video_manager.disconnect(client_id)