class Device:
    """Device class. Represents a device in the environment."""
    
    def __init__(self, id=0, type="SENSOR", position=(0, 0), state="IDLE", max_battery_level_mah=1000):
        """Device class constructor

        Keyword Arguments:
            id {int} -- Device id (default: {0})
            type {str} -- Device type (default: {"SENSOR"})
            position {tuple} -- Device position (default: {(0, 0)})
            state {str} -- Device state (default: {"IDLE"})
            max_battery_level_mah {int} -- Device max battery level in mAh (default: {1000})
        """

        self.id = id
        self.type = type
        self.battery_level_percent = 100
        self.max_battery_level_mah = max_battery_level_mah
        self.battery_level_mah = self.max_battery_level_mah
        self.position = position
        self.state = "IDLE"
