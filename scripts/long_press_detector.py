"""

"""


class LongPressDetector:
    """
    
    """

    def __init__(
        self,
        long_press_duration,
    ):
        """
        
        """

        # # Private constants:
        self.__LONG_PRESS_DURATION = long_press_duration

        # # Public constants:

        # # Private variables:
        self.__start_time = None
        self.__button_was_pressed = False

        # # Publich variables:

    # # Private methods:

    # # Public methods:
    def check_long_press(
        self,
        button_state,
        current_time,
    ):
        """
        
        """

        if button_state:
            # The button was pressed initially.
            if not self.__button_was_pressed:
                self.__button_was_pressed = True
                self.__start_time = current_time

                return False

            # The button is being held.
            if current_time - self.__start_time < self.__LONG_PRESS_DURATION:
                return False

            return True

        # The button was released.
        if self.__button_was_pressed:
            self.__button_was_pressed = False
            self.__start_time = None

        return False
