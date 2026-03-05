from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from ..robot_client import robot_manager
from ..command import COMMAND as cmd

router = APIRouter(prefix="/commands", tags=["commands"])

class CommandRequest(BaseModel):
    speed: Optional[int] = 50

@router.post("/move/{direction}")
async def move(direction: str, req: CommandRequest):
    robot = robot_manager.get_robot()
    if not robot or not robot.connected:
        raise HTTPException(400, "Robot not connected")
    
    dir_map = {
        "forward": cmd.CMD_MOVE_FORWARD,
        "backward": cmd.CMD_MOVE_BACKWARD,
        "left": cmd.CMD_MOVE_LEFT,
        "right": cmd.CMD_MOVE_RIGHT,
        "stop": cmd.CMD_MOVE_STOP
    }
    
    if direction in dir_map:
        command = f"{dir_map[direction]}#{req.speed}\n"
        robot.send_command(command)
        return {"status": "ok", "command": direction}
    
    raise HTTPException(400, "Invalid direction")

@router.post("/hands")
async def hands_up():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_HANDS_UP}\n")
    return {"status": "ok"}

@router.post("/music/play")
async def music_play():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_MUSIC_PLAY}\n")
    return {"status": "ok"}

@router.post("/music/stop")
async def music_stop():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_MUSIC_STOP}\n")
    return {"status": "ok"}

@router.post("/radio/play")
async def radio_play():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_RADIO_PLAY}\n")
    return {"status": "ok"}

@router.post("/radio/stop")
async def radio_stop():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_RADIO_STOP}\n")
    return {"status": "ok"}

@router.post("/function/{num}")
async def function(num: int):
    robot = robot_manager.get_robot()
    if not robot or not robot.connected:
        raise HTTPException(400, "Robot not connected")
    
    func_map = {
        1: cmd.CMD_FUNC_1,
        2: cmd.CMD_FUNC_2,
        3: cmd.CMD_FUNC_3,
        4: cmd.CMD_FUNC_4
    }
    
    if num in func_map:
        robot.send_command(f"{func_map[num]}\n")
        return {"status": "ok"}
    
    raise HTTPException(400, "Invalid function")

@router.post("/look/stop")
async def look_stop():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_LOOK_STOP}\n")
    return {"status": "ok"}

@router.post("/sonic")
async def sonic():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_SONIC}\n")
    return {"status": "ok"}

@router.post("/video/toggle")
async def video_toggle(state: bool):
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        val = "1" if state else "0"
        robot.send_command(f"{cmd.CMD_VIDEO}#{val}\n")
    return {"status": "ok"}

@router.post("/menu")
async def menu():
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_MENU}\n")
    return {"status": "ok"}

@router.post("/about/{num}")
async def about(num: int):
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        robot.send_command(f"{cmd.CMD_FUNC_ABOUT}#{num}\n")
    return {"status": "ok"}