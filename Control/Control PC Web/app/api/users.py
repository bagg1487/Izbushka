from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.robot_client import robot_manager
from app.command import COMMAND as cmd

router = APIRouter(prefix="/users", tags=["users"])

class AddUserRequest(BaseModel):
    name: str

@router.post("/add")
async def add_user(req: AddUserRequest):
    robot = robot_manager.get_robot()
    if not robot or not robot.connected:
        raise HTTPException(400, "Robot not connected")
    
    robot.send_command(f"{cmd.CMD_ADD_USER}#1\n")
    return {"status": "adding_user"}

@router.post("/photo")
async def take_photo(req: AddUserRequest):
    robot = robot_manager.get_robot()
    if not robot or not robot.connected:
        raise HTTPException(400, "Robot not connected")
    
    if not req.name:
        raise HTTPException(400, "Name required")
    
    robot.send_command(f"{cmd.CMD_TAKE_PHOTO}#{req.name}\n")
    return {"status": "photo_taken"}