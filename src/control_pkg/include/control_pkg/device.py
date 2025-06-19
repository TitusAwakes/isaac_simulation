class Point:
    def __init__(self, x, y, z):
        self.x = x 
        self.y = y 
        self.z = z 

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x 
        self.y = y 
        self.z = z 
        self.w = w 

class Device:
    def __init__(self, device_id: str, point: Point, quaternion: Quaternion, battery_level: float):
        self.device_id = device_id
        self.point = point 
        self.quaternion = quaternion
        self.battery_level = battery_level
        