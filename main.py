# main.py
from display import create_lcd_display
from battery_monitor import create_battery_monitor, BatteryMonitor
from gps_simple import GPS_SIMPLE
from time import sleep, ticks_ms
from machine import reset, UART, Pin, I2C, ADC
from rotary_encoder import RotaryEncoder
from mpu6050 import MPU6050
import sys
import time

from neopixel import NeoPixel
########
from uthingsboard.client import TBDeviceMqttClient
import gc
import secrets

########
PIN_ENC_A = 36
PIN_ENC_B = 39
PIN_BUTTON = 12
encoder = RotaryEncoder(PIN_ENC_A, PIN_ENC_B, PIN_BUTTON)

reset_act_not = Pin(0,Pin.IN,Pin.PULL_UP)
#########
Sensorpin = 34
Threshold = 3000
sampling_rate = 20
#########
# Set up ADC for Pulse Sensor
pulse_sensor = ADC(Pin(Sensorpin))
pulse_sensor.atten(ADC.ATTN_11DB)  # Full range: 0-3.6V (if needed)
pulse_sensor.width(ADC.WIDTH_12BIT)  # 12-bit resolution
#########
last_peak_time = 0
bpm = 0
peak_detected = False
#########
Totalkal=0
seckal=0
#########
n = 12
p = 14
np = NeoPixel( Pin (p ,Pin.OUT),n)
#########
lcd_display = create_lcd_display()
battery = create_battery_monitor()
#########
gps_port = 2
gps_speed = 9600

uart = UART(gps_port, gps_speed)
gps = GPS_SIMPLE(uart)
##########
i2c = I2C(0, freq=100000)
imu = MPU6050(i2c)
##########
client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token = secrets.ACCESS_TOKEN)
client.connect()
print("connected to thingsboard, starting to send and receive data")
##########

def set_color(r,g,b):
    for i in range(n):
        np[i] = (r,g,b)
    np.write()
    
def clear_light():
    for j in range(n):
        np[j] = (0,0,0)
    np.write()

    
    
prev_acceleration_x = None
prev_acceleration_y = None
prev_inactivity_notified = None

movement_threshold = 300
inactivity_duration = 5000 #5sec

last_motion_time = ticks_ms()
inactivity_notified = False
############################

weight = encoder.get_value()
print(f"Weight: {weight} kg")
lcd_display.display_message(0,0, f"Weight: {weight} kg")

while True:
    if encoder.adjust_value():
        weight = encoder.get_value()
        if encoder.is_button_pressed():
            print(weight)
            print("Button Pressed! Exiting the program.")
            break  # Exit the loop when the button is pressed
        
start_time = ticks_ms()
loop_treshold = 2000 
acceleration_x = 500
while True:
    
    try:
        signal = pulse_sensor.read()  # Read analog signal
        current_time = time.ticks_ms()  # Current time in milliseconds
        
        if signal > Threshold:
            if not peak_detected:  # Prevent multiple detections for the same peak
                peak_detected = True

                
                # Calculate time since last peak
                if last_peak_time != 0:
                    time_interval = time.ticks_diff(current_time, last_peak_time) / 1000  # Convert ms to seconds
                    bpm = 60 / time_interval
                    
                    if bpm < 89: # relaxed
                        MET = 2
                    elif bpm < 120:  # Light pulse (90-119 bpm)
                        MET = 5
                    elif bpm < 150:  # Moderate pulse (120-149 bpm)
                        MET = 9
                    elif bpm <171:  # Intense cycling (150-170 bpm)
                        MET = 14
                    else:  # Very intense cycling (171-190 bpm)
                        MET = 16
                        
                    print("BPM:", int(bpm))
                    #lcd_display.display_message(2,0, f"BPM:{int(bpm)}    ")
                    #print(weight)
                    
                    
                    # Calculate calories burned per second
                    calories_per_second = ((MET * weight) / 3600)# * time_interval  # Calories per second
                    
                    # Accumulate the total calories burned every second
                    Totalkal += calories_per_second
                    
                    print(f"Calories burned per second: {calories_per_second:.4f}")
                    print(f"Total calories burned: {Totalkal:.2f}")
                    #lcd_display.display_message(2,8, f"TotKcal:{Totalkal:.2f}")
                    #print (MET)
                    
                
                last_peak_time = current_time
        else:
            peak_detected = False
            
        time.sleep(sampling_rate / 1000) # Delay to match sampling rate
        
        if acceleration_x > 0 :
            set_color(220,0,0)
        else:
            clear_light()
        
        if ticks_ms() - start_time > loop_treshold:
            
            lcd_display.lcd.clear()
            
            if gps.receive_nmea_data():
                if gps.get_validity() == "A":
                    latitude = gps.get_latitude()
                    lcd_display.display_message(0, 0, f"N:{latitude:.2f}")
                    longtitute = gps.get_longitude()
                    lcd_display.display_message(0,8, f"E:{gps.get_longitude():.2f}")
                    speed = gps.get_speed()
                    lcd_display.display_message(1,0, f"km/t {speed:.2f}")
                    course = gps.get_course()
                    lcd_display.display_message(1,10, f"C:{gps.get_course():.2f}\n")
                    
                    lcd_display.display_message(2,0, f"BPM:{int(bpm)} ")
                    
                    lcd_display.display_message(2,7, f"TotKcal:{Totalkal:.2f}")
            
            def update_display(inactivity_notified):
                if inactivity_notified:
                    lcd_display.display_message(0, 18, "NM")
                    #print("NOT ACTIVITY")
                else:
                    lcd_display.display_message(0, 18, " M")
                    #print("ACTIVITY")

            def check_motion(acceleration_x, acceleration_y, prev_acceleration_x, prev_acceleration_y, movement_threshold):
                """Checks for significant motion based on acceleration changes."""
                if prev_acceleration_x is None or prev_acceleration_y is None:
                    return False  # No previous values to compare
                delta_x = abs(acceleration_x - prev_acceleration_x)
                delta_y = abs(acceleration_y - prev_acceleration_y)
                return delta_x >= movement_threshold or delta_y >= movement_threshold

            def handle_inactivity(current_time, last_motion_time, inactivity_duration, inactivity_notified):
                """Handles inactivity detection."""
                if current_time - last_motion_time >= inactivity_duration:
                    return True  # Inactivity detected
                return inactivity_notified

            def handle_activity_transition(inactivity_notified, prev_inactivity_notified):
                """Handles the transition from inactivity to activity."""
                if prev_inactivity_notified and not inactivity_notified:
                    print("Transition from No Activity to Activity")
                    # Trigger message sending or other actions here
                    print("send message #######################")
                    send = 1
                    telemetry2 = {"status":send} 
                    client.send_telemetry(telemetry2)
            values = {"acceleration x":0,"acceleration y":0, "temperature celsius":0}
            try:
                values = imu.get_values()
            except:
                print('Error reading imu')
            acceleration_x = values["acceleration x"]
            acceleration_y = values["acceleration y"]
            #print(acceleration_x)            
            
            update_display(inactivity_notified)
            
            if check_motion(acceleration_x, acceleration_y, prev_acceleration_x, prev_acceleration_y, movement_threshold):
                last_motion_time = ticks_ms()
                if inactivity_notified:
                    inactivity_notified = False  # Reset inactivity notification on activity detection
            else:
                current_time = ticks_ms()
                inactivity_notified = handle_inactivity(current_time, last_motion_time, inactivity_duration, inactivity_notified)
                
            if reset_act_not.value()==0:
                print("Reset notifikation")
                send = 0
                telemetry3 = {"status":send}
                client.send_telemetry(telemetry3)
            # Handle transition from inactivity to activity
            handle_activity_transition(inactivity_notified, prev_inactivity_notified)

            # Update previous state and values for the next iteration
            prev_inactivity_notified = inactivity_notified
            prev_acceleration_x = acceleration_x
            prev_acceleration_y = acceleration_y
                
            read_battery = battery.read_battery_percentage()
            lcd_display.display_message(3,0, f"{read_battery}%")
            life = battery.estimate_lifetime()
            lcd_display.display_message(3,5, f"{life}H")
            temp = values["temperature celsius"]
            lcd_display.display_message(3,13,f"{int(temp)} C")
            
            prev_acceleration_x = acceleration_x
            prev_acceleration_y = acceleration_y
            
            latitude = str(gps.get_latitude())
            longtitute = str(gps.get_longitude())
            speed = gps.get_speed()
            course = gps.get_course()
            int(temp)
            telemetry= {
                'battery':read_battery,
                'lat':latitude,
                'lon':longtitute,
                'life':life,
                'speed':speed,
                'course':course,
                'celcius':temp,
                "Totalkalorie":Totalkal
                }
            client.send_telemetry(telemetry)
            
            if gc.mem_free() < 2000:
                print("Garbage collected!")
                gc.collect()
                
            start_time = ticks_ms()

    except KeyboardInterrupt:
        print("Ctrl+C pressed - exiting program.")
        client.disconnect()
        break