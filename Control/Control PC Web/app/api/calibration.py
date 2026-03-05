from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from enum import Enum

from ..robot_client import robot_manager
from ..command import COMMAND as cmd

router = APIRouter(prefix="/calibration", tags=["calibration"])

class LegType(str, Enum):
    L = "L"
    R = "R"
    H = "H"
    M = "M"

class ServoType(str, Enum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"

class CalibrationData(BaseModel):
    leg: LegType
    servo: ServoType
    value: int

@router.get("/data")
async def get_calibration():
    robot = robot_manager.get_robot()
    if not robot or not robot.connected:
        raise HTTPException(400, "Robot not connected")
    
    return {
        "L": robot.calibration_data[0],
        "R": robot.calibration_data[1],
        "H": robot.calibration_data[2],
        "M": robot.calibration_data[3]
    }

@router.post("/set")
async def set_calibration(data: CalibrationData):
    robot = robot_manager.get_robot()
    if not robot or not robot.connected:
        raise HTTPException(400, "Robot not connected")
    
    leg_map = {"L": 0, "R": 1, "H": 2, "M": 3}
    servo_map = {"A": 0, "B": 1, "C": 2, "D": 3, "E": 4}
    
    leg_idx = leg_map[data.leg.value]
    servo_idx = servo_map[data.servo.value]
    
    robot.calibration_data[leg_idx][servo_idx] = str(data.value)
    
    command = f"{cmd.CMD_CALIBRATION}#{data.leg.value}#{data.servo.value}#{data.value}\n"
    robot.send_command(command)
    
    return {"status": "ok"}

@router.post("/mode/{state}")
async def calibration_mode(state: bool):
    robot = robot_manager.get_robot()
    if robot and robot.connected:
        val = "1" if state else "0"
        robot.send_command(f"{cmd.CMD_CALIBRATION_MOD}#{val}\n")
    return {"status": "ok"}