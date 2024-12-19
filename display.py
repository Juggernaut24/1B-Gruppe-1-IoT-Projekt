#display.py
from machine import Pin
from gpio_lcd import GpioLcd


def create_lcd_display():
    return GpioLcdDisplay(
        rs_pin=27, enable_pin=25,
        d4_pin=33, d5_pin=32, d6_pin=21, d7_pin=22,
        num_lines=4, num_columns=20)


class GpioLcdDisplay:
    def __init__(self, rs_pin, enable_pin, d4_pin, d5_pin, d6_pin, d7_pin, num_lines, num_columns):
        """
        Initialize the LCD display.
        """
        self.lcd = GpioLcd(rs_pin=Pin(rs_pin), enable_pin=Pin(enable_pin),
                           d4_pin=Pin(d4_pin), d5_pin=Pin(d5_pin),
                           d6_pin=Pin(d6_pin), d7_pin=Pin(d7_pin),
                           num_lines=num_lines, num_columns=num_columns)
        self.lcd.clear()

    def create_custom_char(self, location, char_map):
        """
        Create a custom character.

        Args:
            location (int): Memory location (0-7).
            char_map (bytearray): Custom character byte pattern (5x8).
        """
        if not (0 <= location <= 7):
            raise ValueError("Custom character location must be between 0 and 7")
        if len(char_map) != 8:
            raise ValueError("Character map must have exactly 8 rows")
        
        self.lcd.custom_char(location, char_map)

    def display_message(self, row, col, message):
        """
        Display a message at a specific position.

        Args:
            row (int): The row number (0-indexed).
            col (int): The column number (0-indexed).
            message (str): The message to display.
        """
        self.lcd.move_to(col, row)
        self.lcd.putstr(message)

    def display_custom_char(self, row, col, location):
        """
        Display a custom character at a specific position.

        Args:
            row (int): The row number (0-indexed).
            col (int): The column number (0-indexed).
            location (int): The memory location of the custom character.
        """
        self.lcd.move_to(col, row)
        self.lcd.putchar(chr(location))

    def clear_display(self):
        """
        Clear the LCD display.
        """
        self.lcd.clear()
        
        