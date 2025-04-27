from battery import BatteryModel

class Device:
    """Device class. Represents a device in the environment."""
    
    def __init__(self, battery_model_idle, battery_model_active, id=0, type="SENSOR", position=(0, 0), state="IDLE", max_battery_level_mah=1000):
        """Device class constructor. Battery model is used to calculate the battery level of the device.

        Keyword Arguments:
            id {int} -- Device id (default: {0})
            type {str} -- Device type (default: {"SENSOR"})
            position {tuple} -- Device position (default: {(0, 0)})
            state {str} -- Device state (default: {"IDLE"})
            max_battery_level_mah {int} -- Device max battery level in mAh (default: {1000})
            battery_model_idle {BatteryModel} -- Battery model for idle state (default: {None})
            battery_model_active {BatteryModel} -- Battery model for active state (default: {None})
        """

        self.id = id
        self.type = type
        self.battery_level_percent = 100
        self.max_battery_level_mah = max_battery_level_mah
        self.battery_level_mah = self.max_battery_level_mah
        self.position = position
        self.state = "IDLE"
        self.battery_model_idle = battery_model_idle
        self.battery_model_active = battery_model_active

        def step(self):
            """Step function. Updates the device state."""
            if self.state == "IDLE":
                self.battery_level_mah -= self.battery_model_idle.delta_battery(self.battery_level_mah)
            elif self.state == "ACTIVE":
                self.battery_level_mah -= self.battery_model_active.delta_battery(self.battery_level_mah)
            else:
                raise ValueError("Invalid state")
            
            if self.battery_level_mah < 0:
                self.battery_level_mah = 0
                self.battery_level_percent = (self.battery_level_mah / self.max_battery_level_mah) * 100
                return -1 # Device is out of battery
            else:
                return 0 # Device is still alive
        

    

