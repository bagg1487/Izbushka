from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional

from app.auth import get_current_user
from app.command import COMMAND as cmd

router = APIRouter(prefix="/commands", tags=["commands"])

class CommandRequest(BaseModel):
    speed: Optional[int] = 50

@router.post("/move/{direction}")
async def move(direction: str, req: CommandRequest, current_user = Depends(get_current_user)):
    if not robot_connected:
        raise HTTPException(400, "Robot not connected")
    
    dir_map = {
        "forward": cmd.CMD_MOVE_FORWARD,
        "backward": cmd.CMD_MOVE_BACKWARD,
        "left": cmd.CMD_MOVE_LEFT,
        "right": cmd.CMD_MOVE_RIGHT,
        "stop": cmd.CMD_MOVE_STOP
    }
    
    if direction in dir_map:
        command = f"{dir_map[direction]}#{req.speed}"
        send_command(command)
        return {"status": "ok", "command": direction}
    
    raise HTTPException(400, "Invalid direction")

@router.post("/hands")
async def hands_up(current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_HANDS_UP}")
    return {"status": "ok"}

@router.post("/music/play")
async def music_play(current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_MUSIC_PLAY}")
    return {"status": "ok"}

@router.post("/music/stop")
async def music_stop(current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_MUSIC_STOP}")
    return {"status": "ok"}

@router.post("/radio/play")
async def radio_play(current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_RADIO_PLAY}")
    return {"status": "ok"}

@router.post("/radio/stop")
async def radio_stop(current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_RADIO_STOP}")
    return {"status": "ok"}

@router.post("/menu")
async def menu(current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_MENU}")
    return {"status": "ok"}

@router.post("/about/{num}")
async def about(num: int, current_user = Depends(get_current_user)):
    if robot_connected:
        send_command(f"{cmd.CMD_FUNC_ABOUT}#{num}")
    return {"status": "ok"}