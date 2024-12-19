# rotary_encoder.py
from machine import Pin
from display import create_lcd_display
lcd_display = create_lcd_display()

class RotaryEncoder:
    CW = 1  # Clockwise
    CCW = -1  # Counter-clockwise

    def __init__(self, pin_a, pin_b, button_pin, min_value=30, max_value=200, initial_value=70):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.button = Pin(button_pin, Pin.IN, Pin.PULL_UP)

        self.min_value = min_value
        self.max_value = max_value
        self.value = initial_value
        self.enc_state = 0

        self.enc_table_full_step = [
            [0x00, 0x02, 0x04, 0x00],
            [0x03, 0x00, 0x01, 0x10],
            [0x03, 0x02, 0x00, 0x00],
            [0x03, 0x02, 0x01, 0x00],
            [0x06, 0x00, 0x04, 0x00],
            [0x06, 0x05, 0x00, 0x20],
            [0x06, 0x05, 0x04, 0x00]
        ]

    def read_encoder(self):
        self.enc_state = self.enc_table_full_step[
            self.enc_state & 0x0F
        ][(self.pin_b.value() << 1) | self.pin_a.value()]

        result = self.enc_state & 0x30
        if result == 0x10:
            return self.CW
        elif result == 0x20:
            return self.CCW
        else:
            return 0

    def adjust_value(self):
        direction = self.read_encoder()
        if direction == self.CW:
            if self.value < self.max_value:
                self.value += 1
                print(f"Weight: {self.value} kg")
                lcd_display.display_message(0, 0, f"Weight: {self.value} kg")
        elif direction == self.CCW:
            if self.value > self.min_value:
                self.value -= 1
                print(f"Weight: {self.value} kg")
                lcd_display.display_message(0, 0, f"Weight: {self.value} kg")
                
        return self.is_button_pressed()
                

    def is_button_pressed(self):
        return self.button.value() == 0

    def get_value(self):
        return self.value
