#!/usr/bin/env python3

class IncorrectCozmoVersionError(Exception):
    """Exception raised for errors in the Cozmo firmware version.

    Attributes:
        version -- Cozmo firmware version that caused the error
        message -- explanation of the error
    """

    def __init__(self, version, message=""):
        self.version = version
        self.message = "Cozmo firmware is currently version {}, and must be updated to version 2381 to operate correctly.".format(self.version)
        if message is not "":
            self.message = message
        super().__init__(self.message)