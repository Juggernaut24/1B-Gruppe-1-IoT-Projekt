from machine import ADC, Pin, I2C
from ina219_lib import INA219
from time import sleep


def create_battery_monitor():
    return BatteryMonitor(adc_pin=35, divider_ratio=1.75, min_voltage=3.0,
                          max_voltage=4.2,i2c_port=0, ina219_i2c_addr=0x40)

class BatteryMonitor:
    def __init__(self, adc_pin, divider_ratio, min_voltage, max_voltage, i2c_port=0, ina219_i2c_addr=0x40):
        # Konfiguration
        self.divider_ratio = divider_ratio
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        
        # Opsætning af ADC
        self.adc = ADC(Pin(adc_pin))
        self.adc.atten(ADC.ATTN_11DB)  # Til måling op til 3.3V
        
        # Opsætning af INA219
        self.i2c = I2C(i2c_port)
        self.ina219 = INA219(self.i2c, ina219_i2c_addr)
        self.ina219.set_calibration_16V_400mA()

    def read_battery_percentage(self):
        # Læs rå ADC værdi
        raw_value = self.adc.read()
        
        U2 = 4 #y2
        adc2 = 2659 #x2
        U1 = 3 #y1
        adc1 = 1965 #x1

        a = (U2-U1)/(adc2-adc1) # 0.0016-0.0017
        b = U2-a*adc2 # 0.3
        
        u_batt = a* raw_value + b
        
        without_offset = u_batt - 3.0
        normalized = without_offset / 1.2 #4.2V-3.0V = 1.2V
        percentage = normalized * 100
        
        return int(percentage)
        

    def estimate_lifetime(self):
        # Beregn resterende batterilevetid i timer
        current = self.ina219.get_current()
        if current > 0:
            capacity_remaining = 1800 * (self.read_battery_percentage() / 100)  # Kapacitet i mAh
            lifetime = capacity_remaining / current  # Tid i timer
            return int(lifetime)
        else:
            return float('inf')  # Hvis strømforbrug er 0, er batteritiden uendelig

# Brug biblioteket
if __name__ == "__main__":
    monitor = BatteryMonitor(adc_pin=35, divider_ratio=1.75, min_voltage=3.0, max_voltage=4.2,
                             i2c_port=0, ina219_i2c_addr=0x40)
    
    while True:
        battery_percentage = monitor.read_battery_percentage()
        raw_value = monitor.adc.read()
        print(raw_value)
        print(raw_value / 4095 * 3.3)
        lifetime = monitor.estimate_lifetime()
        print(f"Batteri: {battery_percentage}%")
        print(f"levetid: {int(lifetime)} timer")
        sleep(1)
