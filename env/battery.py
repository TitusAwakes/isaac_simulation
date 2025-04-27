class BatteryModel:

    """Battery model for recharging and discharging. Every type of device has a different battery model.
    The battery model is divided into 3 phases: exponential, linear and rational.

    Returns:
        BatteryModel -- Battery model for recharging and discharging
    """

    def __init__(self, end_of_exponential_phase_percentage, end_of_linear_phase_percentage, end_of_rational_phase_percentage,
                 battery_delta_exponential, battery_delta_linear, battery_delta_rational):
        """ Battery model for recharging and discharging. Every type of device has a different battery model.
        The battery model is divided into 3 phases: exponential, linear and rational.
        Battery deltas are the amount of battery that is consumed or recharged in each phase, depending on what the model is used for.

        Arguments:
            end_of_exponential_phase_percentage {int} -- Percentage of battery level when the exponential phase ends
            end_of_linear_phase_percentage {int} -- Percentage of battery level when the linear phase ends
            end_of_rational_phase_percentage {int} -- Percentage of battery level when the rational phase ends
            battery_delta_exponential {int} -- Battery delta in the exponential phase
            battery_delta_linear {int} -- Battery delta in the linear phase
            battery_delta_rational {int} -- Battery delta in the rational phase
        """


        self.end_of_exponential_phase_percentage = end_of_exponential_phase_percentage
        self.end_of_linear_phase_percentage = end_of_linear_phase_percentage
        self.end_of_rational_phase_percentage = end_of_rational_phase_percentage
        self.battery_delta_exponential = battery_delta_exponential
        self.battery_delta_linear = battery_delta_linear
        self.battery_delta_rational = battery_delta_rational

    def delta_battery(self, device_battery_level):

        """Deltas battery according to the battery model
        
        Arguments:
            device_battery_level {int} -- Device battery level

        Returns:
            int -- New device battery level

        """

        if device_battery_level <= self.end_of_exponential_phase_percentage:
            return self.__exponential_phase(device_battery_level)
        elif device_battery_level <= self.end_of_linear_phase_percentage:
            return self.__linear_phase(device_battery_level)
        elif device_battery_level <= self.end_of_rational_phase_percentage:
            return self.__rational_phase(device_battery_level)
    
    def __exponential_phase(self, device_battery_level):

        """Deltas battery according to the exponential phase

        Arguments:
            device_battery_level {int} -- Device battery level

        Returns:
            int -- New device battery
        """

        return device_battery_level - self.battery_delta_exponential
    
    def __linear_phase(self, device_battery_level):
        """Deltas battery according to the linear phase

        Arguments:
            device_battery_level {int} -- Device battery level

        Returns:
            int -- New device battery
        """

        return device_battery_level - self.battery_delta_linear
    
    def __rational_phase(self, device_battery_level):
        """Deltas battery according to the rational phase

        Arguments:
            device_battery_level {int} -- Device battery level

        Returns:
            int -- New device battery
        """

        return device_battery_level - self.battery_delta_rational

