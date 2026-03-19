import random

class Robot:
    def __init__(self):
        self.connected = True
        self.calibration_data = [["0"]*5 for _ in range(4)]
        self.sonic_value = "0"
    
    def send_command(self, cmd):
        print(f"🤖 Robot received: {cmd.strip()}")
        if "CMD_SONIC" in cmd:
            self.sonic_value = str(random.randint(10, 100))
    
    def get_sonic(self):
        return self.sonic_value

class RobotManager:
    def __init__(self):
        self._robot = None
    
    def get_robot(self):
        if self._robot is None:
            self._robot = Robot()
        return self._robot

robot_manager = RobotManager()