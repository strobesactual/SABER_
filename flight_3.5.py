""" 

Notes:
1) To reduce servo jitter, use the pigpio pin driver rather than the default RPi.GPIO driver 
    (pigpio uses DMA sampling for much more precise edge timing)
    
Changes needed:
1) Run on startup
2) Collision lights on dusk to dawn only and for 2 minutes post launch
3) Automatic nohup
4) Monitor GPS every 5 seconds or more
5) Spend 95% of time listening for radio transmissions
x) Get the DHT-22 working
x) Get the Baro working
3) Fold in code to set the airborne mode to Airborne <2G at the beginning



x) Charge function is not working

sudo pip install LoRaRF --break-system-packages

Path to python interpreter: /usr/bin/python3
Path to file: /home/11a/flight_3.4.py


"""


import asyncio
import csv
from datetime import datetime, date
import gpiozero
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory 
import os
import re

from ina219 import INA219
from ina219 import DeviceRangeError
from LoRaRF import SX127x
from pigpio_dht import DHT22

import pynmea2
import serial
from shapely.geometry import Polygon, Point
import subprocess
import sys
import time


#-------------------- INPUT REQUIRED --------------------
# CONFIGURE NEO6M with the computer if the code wont do it

flight_time_limit = 7200    # 3600 = 1h; 5400 = 90m; 7200 = 2h
record_interval = 10        # seconds between recording flight data
display_interval = 10       # Seconds between data being displayed to the screen. !! MUST be LONGER thank sensor_interval
sensor_interval = 20         # Seconds between sensor readings
heat_time = 12              # Seconds Nichrome plate is heating 
airborne_delta = 20         # Meters above launch site elevation to trigger airborne mode

update_interval = 2         # Minutes between updates
msg_iterations = 2          # Total number of times the message will be sent per update burst
msg_interval = 5            # Number of seconds between the burst messages (>=5 required)

descent_threshold = 1500    # Meters above sea level to trigger descent



# ***** This needs to be confirmed prior to Wednesday *****
test_area = "Colorado Springs--Falcon Regional (elev: 2395 M)"
bound_1 = Point(41.000187, -102.050539) #NE Corner
bound_2 = Point(36.988411, -102.038130) #SE Corner
bound_3 = Point(37.000241, -109.040373) #SW Corner
bound_4 = Point(41.008131, -109.054018) #NW Corner


"""
test_area = "California--Skylark Park (Elevation 32m)"
bound_1 = Point(37.06231877848365, -120.34638618916614) #SE Corner
bound_2 = Point(37.49052266840307, -120.69282482882616) #NE Corner
bound_3 = Point(37.47952653379495, -121.10970599188374) #NW Corner
bound_4 = Point(37.01622957747398, -120.91339076274305) #SW Corner
"""


#-------------------- FORMATTING & ID --------------------
(RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, MAGENTA, RESET) = ('\033[91m', '\033[38;5;208m', '\033[93m', '\033[92m', '\033[96m', '\033[94m', '\033[95m', '\033[0m')
print(f'\n{CYAN}{"Initializing..."}{RESET}\n')

def get_computer_type():
    global balloon_id
    username = os.getlogin()
    match = re.match(r"(\w+)", username)     # Extract the username part using regular expression
    if match:
        username = match.group(1)
        last_letter = username[-1]      # Get the last letter of the username
        is_primary = last_letter == "a"    # Check if the username is the primary user
        balloon_id = username
    else:
        is_primary = False
    return is_primary

primary = get_computer_type()
print(f"{MAGENTA}{'Computer type:':<20}{RESET}{'Primary' if primary else 'Backup'}\n")


#-------------------- I2C SETUP --------------------
if primary:
    SHUNT_OHMS = 0.1
    MAX_EXPECTED_AMPS = 0.4
    I2C_BUS = 1

    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40, busnum=I2C_BUS)
    ina.configure(ina.RANGE_16V, ina.GAIN_1_40MV, ina.ADC_128SAMP, ina.ADC_128SAMP)
else:
    ina = None


#-------------------- SERIAL PORT SETUP --------------------
port_used = None
def find_available_port(ports):
    for port in ports:
        try:
            with serial.Serial(port, baudrate=9600, timeout=1) as ser:
                if ser.readline():
                    return port
        except Exception as e:
            pass
        time.sleep(1)
    raise RuntimeError()

try:
    port_used = find_available_port(['/dev/ttyS0', '/dev/ttyAMA0'])
except RuntimeError as e:
    print(f'{RED}{"Serial port error:":<25}{RESET}{e}')
    sys.exit()
    
ser = serial.Serial(port_used, baudrate=9600, timeout=0.5)
dataout = pynmea2.NMEAStreamReader()
newdata = ser.readline()


#-------------------- GPIO SETUP --------------------
command = "sudo pigpiod"    # Is this even required to run? <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
try:
    result = subprocess.run(command, shell=True, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.stderr:
        status_message = f'{RED}{"Failed     "}{RESET}'
        error_message = result.stderr.strip()
    else:
        status_message = f'{"Initialized"}'
        error_message = ''  # No error message
except subprocess.CalledProcessError as e:
    status_message = f'{RED}{"Failed     "}{RESET}'
    error_message = e.stderr.strip()

relay_on = 1  # The HiLetgo uses standard 0 = Off and 1 = On, but the ELEGOO relay uses 1 OFF and 0 on
relay_off = 0
strobe_led = gpiozero.PWMOutputDevice(5)
strobe_led.value = relay_off 
status_led = gpiozero.PWMOutputDevice(6)
status_led.value = relay_off 
heat_element = gpiozero.PWMOutputDevice(26)
heat_element.value = relay_off 
if primary:
    servo = Servo(4, pin_factory=PiGPIOFactory()) # Pin 7
    servo_open = 1
    servo_close = -1
    servo.value = servo_close
    sensor_dht22 = DHT22(16)
else:
    sensor_dht22 = None 


#-------------------- INITIALIZE GLOBAL VARIABLES --------------------
gps_lat = 0.0
gps_lon = 0.0
gps_spd = 0
gps_trk = 0
gps_day = '00'
gps_alt = 0.0
gps_sat = '0'
gps_time = ''
gps_valid = False
map_link = "*** No GPS data ***"

base_alt = 0
base_set = False
max_alt = 0
descent_alt = 0
#climb_iteration = 0
descent_iteration = 0
#cruise_iteration = 0
#climbing = False
#cruising = False
descending = False


airborne = False
terminate = False
intact = True 
trigger = ''

polygon = Polygon([bound_1, bound_2, bound_3, bound_4])
contained = False

int_temp = 0.0
int_humid = 0.0
voltage = 0.0
current = 0.0
power = 0.0
charge = 0.0
baro_alt = 0

start_time = None
flight_time = 0
record_time = '' 

msg_sent = ''
object_report = 'None'
tx_counter = 0
send_update = True
tx_time = 0
tx_rate = 0

#-------------------- NEO-6M Initialization --------------------
def neo6m_configure(): 
    print(f'{YELLOW}{"Configure NEO-6M before flight!!!"}{RESET}')
    # Put actual code here to send the config message
    
    
#-------------------- SX1278 Initialization --------------------
LoRa = SX127x()
def configure_sx1278():
    # SPI Port Configuration: bus id 0 and cs id 1 and speed 7.8 Mhz
    LoRa.setSpi(0, 0, 7800000)  
    # I/O Pins Configuration: set RESET->22, BUSY->23, DIO1->26, TXEN->5, RXEN->25
    LoRa.setPins(22, 23)  
    LoRa.begin()
    # Modem Configuration 
    LoRa.setTxPower(17, LoRa.TX_POWER_PA_BOOST)     # Set TX power +17 dBm using PA boost pin
    LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING, LoRa.RX_GAIN_AUTO)    # AGC on, Power saving gain
    LoRa.setFrequency(433500000)                    # Set frequency to 433.775 Mhz, the LoRa APRS frequency, or 433.5
    # Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
    LoRa.setSpreadingFactor(12)     # 12 (max) Prioritizes long-range communication and can tolerate a slower data rate
    LoRa.setBandwidth(125000)       # 125 kHz: most commonly used bandwidth for LoRa and is ideal for long-range communication
    LoRa.setCodeRate(5)   # 8 is best for very noisy or long-range applications where maximum reliability is necessary    
    # Packet configuration 
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 15, True, False)   # set explicit header mode, preamble length 12, payload length 15, CRC on and no invert IQ operation
    # Set syncronize word for public network (0x3444)
    LoRa.setSyncWord(0x2005)    # Set syncronize word for public network (0x3444)
configure_sx1278()

    
#-------------------- Preflight Information --------------------    
print(f"{MAGENTA}{'-' * 100}{RESET}\n"
    f"{CYAN}{'Flight telemetry configuration':<50}{'Preflight data'}{RESET}\n"
    f"{RESET}{'Serial port:':<20}{RESET}{port_used:<30}{'Time:':<20}{RESET}{datetime.now().strftime('%H%M on %d %b')}\n"
    f"{RESET}{'INA219:':<20}{RESET}{'Initialized' if primary else 'Not equipped':<30}{'Balloon ID:':<20}{RESET}{balloon_id}\n"
    f"{RESET}{'PiGPIO:':<20}{RESET}{status_message:<30}{'Computer:':<20}{RESET}{'Primary' if primary else 'Backup'}\n"
    f"{RESET}{'DHT22:':<20}{RESET}{'Not equipped' if sensor_dht22 is None else 'Initialized':<30}{'Test area:':<20}{RESET}{test_area}\n"
    f"{'':<50}{RESET}{'Flight limit (s):':<20}{RESET}{flight_time_limit}\n\n"
   
    f"{CYAN}{'Radio Config':<50}{'Station Info':<20}{RESET}\n"
    f"{'Transmit frequency:':<20}{YELLOW}{LoRa._frequency / 1000000:.3f}{' MHz':<23}{RESET}"
      f"{'Station ID:':<20}{YELLOW}{'NONE'}{RESET}\n"
    f"{'Bandwidth:':<20}{YELLOW}{LoRa._bw / 1000000:>7.3f}{' MHz':<23}{RESET}"
      f"{'Station config:':<20}{YELLOW}{'Balloon'}{RESET}\n"
    f"{'Spreading factor:':<20}{YELLOW}{LoRa._sf:<30}{RESET}\n"
    f"{'Coding rate:':<20}{YELLOW}{LoRa._cr:<30}{RESET}\n"
    f"{MAGENTA}{'-' * 100}{RESET}\n"
)


#-------------------- TROUBLESHOOTING --------------------
def print_mark(mark):  # TROUBLESHOOTING
    global timestamp
    timestamp = datetime.now().strftime("%H:%M:%S") 
    print(MAGENTA, f"Mark {mark}:", timestamp, RESET, '\n')


#-------------------- TELEMETRY --------------------
async def gps():  #~~~~~ TASK 1 ~~~~~
    global gps_valid, gps_lat, gps_lon, gps_spd, gps_trk, gps_time, gps_day, map_link, gps_alt, gps_sat, airborne
    global max_alt, descent_alt
    while True:
        try:
            newdata = ser.readline()
            if newdata[0:6] == b"$GPRMC":
                rmcmsg = pynmea2.parse(newdata.decode('utf-8'))
                gps_valid = True if rmcmsg.status == "A" else False
                gps_lat = float("{:.5f}".format(rmcmsg.latitude))
                gps_lon = float("{:.5f}".format(rmcmsg.longitude))
                gps_spd = float("{:.1f}".format(rmcmsg.spd_over_grnd)) if rmcmsg.spd_over_grnd is not None else 0.0
                gps_trk = float("{:.1f}".format(rmcmsg.true_course)) if rmcmsg.true_course is not None else 0.0
                gps_time = rmcmsg.timestamp.strftime('%H:%M:%S')
                #gps_day = rmcmsg.timestamp.strftime('%d')  # added to get the date for use in the APRS message
                gps_day = str(date.today().day)
                map_link = f'{gps_lat},{gps_lon}'
            if newdata[0:6] == b"$GPGGA":
                ggamsg = pynmea2.parse(newdata.decode('utf-8'))
                gps_alt = float("{:.1f}".format(ggamsg.altitude))
                gps_sat = ggamsg.num_sats
            await asyncio.sleep(.01)  # This must be the smallest sleep in all the code
        except Exception as e:
            print(f"\n{RED}{'GPS data error:':<25}{RESET}{e}\n")
        
 
async def record():  #~~~~~ TASK 2 ~~~~~
    global record_interval, record_time
    filetime = datetime.now().strftime('%d%b_%H%M')  # Format (DDMon_HHMM)
    filename = f"{balloon_id}_flight_data_{filetime}.csv"
    print(f'{MAGENTA}{"Data record created:":<25}{RESET}{filename}\n')
    while True:
        try:        
            with open(filename, mode='a', newline='') as file:  # Open the CSV file in append mode
                csv_writer = csv.writer(file)  # Create a CSV writer object
                if file.tell() == 0:  # Write the headers if the file is empty
                    csv_writer.writerow(["CPU Time", "GPS Time", "Latitude", "Longitude", "Altitude (M)", 
                                         "Track", "Speed (kts)", "Flt mode", "Elapsed (s)", "Contained", 
                                         "Terminate", "Intact", "Trigger", "Int temp", "Int humid", 
                                         "Voltage (V)", "Current (mA)", "Power (mW)"])
                record_time = datetime.now().strftime("%H:%M:%S")
                csv_writer.writerow([record_time, gps_time, gps_lat, gps_lon, gps_alt, 
                                     gps_trk, gps_spd, airborne, flight_time, contained, 
                                     terminate, intact, trigger, int_temp, int_humid, 
                                     voltage, current, power]) 
                # print(f'\n{MAGENTA}{"Data written to CSV:":<25}{RESET}Time {record_time} at {gps_alt}m MSL located: {gps_lat} / {gps_lon} traveling {gps_trk}deg at {gps_spd}kts\n')
            await asyncio.sleep(record_interval)   # Wait for x seconds before writing to the CSV file again
        except Exception as e:
            print(f"\n{RED}{'CSV write error:':<25}{RESET}{e}\n")
  
                
async def display():  #~~~~~ TASK 3 ~~~~~
    global object_report
    while True:
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"{'CPU time:':<18}{MAGENTA}{timestamp:<10}{RESET}{'Local':<10} {'Sunrise:':<18}{'None'}{' UTC':<10} {'Temperature (°C):':<20}{int_temp:<20.1f}")
            print(f"{'GPS time:':<18}{BLUE}{gps_time:<10}{RESET}{'UTC':<10} {'Sunset:':<18}{'None'}{' UTC':<10} {'Humidity (%):':<20}{int_humid:<20.1f}")
            print(f"{'Lat:':<18}{gps_lat:<20.6f} {'Track (°):':<18}{gps_trk:<14} {'Bus Voltage (V):':<20}{voltage:<5.1f}")
            print(f"{'Lng:':<17}{gps_lon:<21.6f} {'Speed (kts):':<18}{gps_spd:<14} {'Bus Current (mA):':<20}{current:<5.1f}") 
            print(f"{'GPS Alt (M):':<18}{gps_alt:<20.1f} {'Satellites:':<18}{gps_sat:<14} {'Bus Power (mW):':<20}{power:<5.0f}")
            print(f"{'GPS Valid:':<18}{GREEN if gps_valid else RED}{'Valid' if gps_valid else 'NO GPS':<21}{RESET}{'Location:':<18}{RESET if gps_valid else RED}http://maps.google.com/?q={map_link}{RESET}")
            print(f"{'Flight status:':<18}{GREEN if airborne else ORANGE}{'Airborne' if airborne else 'Ground':<21}{RESET}{'Geofenced:':<18}{GREEN if contained else RED}{'Contained' if contained else 'OUTSIDE':<21}{RESET}") 
            
            print(f"{'Flight time:':<18}{CYAN}{flight_time:<21}{RESET}{'Time limit:':<18}{flight_time_limit:<20}")
            print(f"{'Trigger:':<18}{ORANGE}{trigger:<21}{RESET}{'Intact:':<18}{'True' if intact else 'False':<20}")
            #print(f"{'Max Alt:':<18}{max_alt:<21}{'Mode:':<18}{'Climbing' if climbing else 'Cruising' if cruising else 'Descending' if descending else 'Ground':<20}")
            print(f"{MAGENTA}{'CSV update:':<18}{GREEN}{record_time}{RESET}")
            print(f"{BLUE}{'Message:':<18}{YELLOW}{object_report}{RESET}")
            print(f"{BLUE}{'Transmit time:':<18}{RESET}{tx_time}{' s'}")
            print(f"{BLUE}{'Data rate:':<18}{RESET}{tx_rate}{' byte/s'}")
            print(f"{BLUE}{'Timestamp:':<18}{YELLOW}{msg_sent}{RESET}\n")
            print(f"{RESET}{'Base Alt:':<18}{RESET}{str(base_alt):<21}")
            print(f"{RESET}{'Descent Alt:':<18}{RESET}{descent_alt:<21}{RESET}{'Max Alt:':<18}{RESET}{max_alt:<21}")
            print(f"{RESET}{'-' * 100}{RESET}")
            await asyncio.sleep(display_interval)   
        except Exception as e:
            print(f"\n{RED}{'Display Error:':<25}{RESET}{e}\n")
            break


async def set_base_alt():
    global base_alt, base_set
    readings = []
    while not base_set:
        start_time = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start_time < 20:
            if gps_valid:
                readings.append(gps_alt)
            await asyncio.sleep(1)  # Collect readings every second
        if readings:
            base_alt = round(sum(readings) / len(readings), 2)
        else:
            base_alt = 0  # Default value if no readings were collected
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"Base Altitude set at {timestamp} to: {base_alt}")
        base_set = True
"""
def assess_climbing():
    global max_alt, climb_iteration
    
    #while True:
    if gps_alt > max_alt:
        climb_iteration += 1
        max_alt = gps_alt
    else:
        climb_iteration = 0
    if climb_iteration > 3:
        return True
    else:
        return False
"""        

def assess_descent():
    global descent_alt, descent_iteration 
    #while True:
    if gps_alt < descent_alt:
        descent_iteration += 1
        descent_alt = gps_alt
    else:
        descent_iteration = 0
        descent_alt = gps_alt
    if descent_iteration > 3:
        return True
    else:
        return False


async def assess_flight():
    global airborne, contained, status_led, strobe_led
    
    while True:
        try:
            status_led.value = relay_on     # For auto-run on power-on, this indicates the code is running
            preflight = True
            good_gps = False    # Sometimes the GPS gets a bad read after an initial signal
            preflight_start = asyncio.get_event_loop().time()
            await set_base_alt()
            if asyncio.get_event_loop().time() - preflight_start > 300:  # 5-minute buffer before airborne mode
                preflight = False
                airborne = True       
            if gps_valid and contained:
                strobe_led.value = relay_on
                good_gps = True
        except Exception as e:
            print(f"\n{RED}{'Flight assessment error:':<25}{RESET}{e}\n")  
                 

async def assess_airborne():  #~~~~~ TASK 4 ~~~~~
    global airborne, contained, status_led, strobe_led, descent_alt, descending
    while True:
        try:
            await asyncio.sleep(sensor_interval)    # If it lands at a lower elevation, the strobes will turn off
            if not airborne:
                await set_base_alt() 
                if gps_valid and contained:
                    status_led.value = relay_on
                    strobe_led.value = relay_off    # not required as the strobes are already off
                    descent_tx = False
                elif gps_alt > (base_alt + airborne_delta):
                    airborne = True
                else:   
                    airborne = False
            else:         
                strobe_led.value = relay_on       # Consider turning on the strobes for 1 minute and then again only at night time
                status_led.value = relay_off
                descending = assess_descent()
                if descending and descent_alt < 3048 and not descent_tx:   # ***** NEW ADDITION TO SEND AN UPDATE ON DESCENT *****
                    await transmit_report()     # 5486M = 18,000ft, 3048M = 10,000ft, 1524M = 5,000ft
                    descent_tx = True
                if gps_alt < 3048 and descent_tx:
                    strobe_led.value = relay_off  
            """else: 
                strobe_led.value = relay_off
                status_led.value = relay_off"""
            
            await asyncio.sleep(3)  # Ensure this sleep time does not interfere with other tasks
        except Exception as e:
            print(f"\n{RED}{'Assessment error:':<25}{RESET}{e}\n")


async def environmental_monitor():  #~~~~~ TASK 5 ~~~~~
    global int_temp, int_humid
    while primary:
        try:
            await asyncio.sleep(sensor_interval)
            result = sensor_dht22.read()
            if 'temp_c' in result and 'humidity' in result:     # Ensure the result contains the expected data
                raw_temp = result['temp_c']
                int_temp = round(-1 * (raw_temp + 3276.8), 1) if raw_temp < 0 else round(raw_temp, 1)  # known error with sub-zero temps
                int_humid = round(float(result['humidity']), 1)     # Directly convert the formatted string to a float
            else:
                print(f"\n{RED}{'DHT-22 data missing':<25}{RESET}\n")   # Handle cases where 'temp_c' or 'humidity' is missing in the result
        except Exception as e:
            print(f"\n{RED}{'DHT-22 error:':<25}{RESET}{e}\n")
        await asyncio.sleep(3)
        #print_mark(5)


async def electrical_monitor():  #~~~~~ TASK 6 ~~~~~
    global voltage, current, power, charge
    while primary:
        try:
            await asyncio.sleep(sensor_interval)
            voltage = ina.voltage()
            current = ina.current()
            power = ina.power()
            
            """
            #Check this in the preflight code prior to putting into use
            voltage = float("{:.1f}".format(ina.voltage))
            current = float("{:.1f}".format(ina.current))
            power = float("{:.1f}".format(ina.power))
            charge = 100 * ((voltage-9.5)/(12.22-9.5)) # full = 12.22v, empty = 9.5v, dead = 8.6v
            """
        except DeviceRangeError as e:
            print(f'\n{RED}{"INA-219 error:":<25}{RESET}{e}\n')
        await asyncio.sleep(10) # Wait 10 sec prior to next reading
        # print_mark(6)


async def baro_monitor():  #~~~~~ TASK 7 ~~~~~   *** THIS IS NOT FUNCTIONAL ***
    global baro_alt
    while primary:
        try:
            await asyncio.sleep(sensor_interval)
            baro_alt = 999
        except Exception as e:
            print(f'\n{RED}{"HX710B error:":<25}{RESET}{e}\n')
        await asyncio.sleep(sensor_interval)
        #print_mark(7)
        
        
# ---------- OBJECT REPORT for the AX.25 INFORMATION FIELD ---------- 
def convert_coordinates(coordinate, is_latitude=True):
    degrees = int(coordinate)       # Extract the degrees part
    minutes = abs(coordinate - degrees) * 60
    
    if is_latitude:
        # Latitude: 2 digits for degrees, 2 for minutes (to two decimal places), followed by N or S
        ddmm = f"{abs(degrees):02d}{minutes:05.2f}"  # 2 digits for degrees, 4 for minutes
        direction = 'N' if coordinate >= 0 else 'S'
        return f"{ddmm}{direction}"  # Total length = 8 characters
    else:
        # Longitude: 3 digits for degrees, 2 for minutes (to two decimal places), followed by E or W
        ddmm = f"{abs(degrees):03d}{minutes:05.2f}"  # 3 digits for degrees, 4 for minutes
        direction = 'E' if coordinate >= 0 else 'W'
        return f"{ddmm}{direction}"  # Total length = 9 characters


async def format_report():
    global object_report
    
    info_field_data_id = ";"                # The ; is the APRS Data Type Identifier for an Object Report
    object_name = "SABER_" + balloon_id     # Fixed 9-character Object name, which may consist of any printable ASCII characters
    alive_killed = '*'                      # a * or _ separates the Object name from the rest of the report '*' = live Object. '_' = killed Object
    aprs_timestamp = f"{gps_day}{gps_time[:2]}{gps_time[3:5]}z"  # 7 bytes (DDHHMMz)
    aprs_lat = convert_coordinates(gps_lat, True)
    sym_table_id = "/"
    aprs_lon = convert_coordinates(gps_lon, False)
    symbol_code = "O"                       # Primary Symbol Table, Balloon = "O" (SSID -11)
    crs_spd = f"{int(gps_trk):03d}/{int(gps_spd):03d}"
    aprs_comment = f"++Alt:{gps_alt}m_{round(flight_time / 60, 1)}min^{'Intact' if intact else 'Killed'}>{trigger}<"   # Max 43 Characters
            
    object_report = (
        f"{info_field_data_id}{object_name}{alive_killed}{aprs_timestamp}"
        f"{aprs_lat}{sym_table_id}{aprs_lon}{symbol_code}{crs_spd}{aprs_comment}"
    )
    return object_report        


async def transmit_report():  
    global tx_counter, msg_sent, object_report, tx_time, tx_rate
    try:
        object_report = await format_report()    
        byte_message = list(object_report.encode('utf-8'))  # Converts the string to a byte array and then into a list of numbers for each byte
        
        LoRa.beginPacket()
        LoRa.write(byte_message, len(byte_message))   # This sends the message, which is now a byte array
        LoRa.write([tx_counter], 1)    # This sends the counter value, which is likely an additional byte appended to the message, perhaps to indicate a message sequence number or packet identifier.
        LoRa.endPacket()
        LoRa.wait()
        
        tx_counter = (tx_counter + 1) % 256
        msg_sent = datetime.now().strftime("%H:%M:%S")
        tx_time = f"{LoRa.transmitTime() / 1000:.2f}"
        tx_rate = f"{LoRa.dataRate():.2f}"
    except Exception as e:
        print(f"\n{RED}{'Transmit Error:':<25}{RESET}{e}\n")
        
        
async def periodic_update():  #~~~~~ TASK 8 ~~~~~
    while True:
        for _ in range(msg_iterations):  # Loop for a fixed number of iterations
            await transmit_report()  # Call the async function
            await asyncio.sleep(msg_interval)  
        print(f"{MAGENTA}{'Messages sent:':<25}{CYAN}{msg_iterations}{RESET} at {msg_sent} on {LoRa._frequency / 1000000:.3f} MHz")
        print(f"{BLUE}{'Transmit time:':<25}{RESET}{LoRa.transmitTime() / 1000:0.2f}{' s'}")
        print(f"{BLUE}{'Data rate:':<25}{RESET}{LoRa.dataRate():0.2f}{' byte/s'}")
        print("----------------------------------------------------------------------------------------------")
        await asyncio.sleep(update_interval * 60)  # Wait before the next update
        
            
#-------------------- TERMINATION --------------------
async def flight_timer(airborne):   #~~~~~ TASK 10 ~~~~~
    global start_time, flight_time, trigger, intact
    while True:
        if airborne() and start_time is None:    
            start_time = asyncio.get_event_loop().time()
        if start_time is not None:
            flight_time = int(asyncio.get_event_loop().time() - start_time)
            if flight_time > flight_time_limit and intact:
                await terminate_balloon()
                trigger = "Timing"
                intact = False 
            else:
                pass
        await asyncio.sleep(5)  # Update every 5 seconds


async def geofencing():  #~~~~~ TASK 11 ~~~~~
    global gps_lat, gps_lon, polygon, contained, trigger, intact
    while True:
        try:
            location = Point(gps_lat, gps_lon)
            contained = polygon.contains(location)
            if gps_valid:
                if not contained:
                    await asyncio.sleep(10) # Wait for 10 seconds to check if the location is still outside
                    # Recheck location after waiting
                    location = Point(gps_lat, gps_lon)
                    contained = polygon.contains(location)
            
                    if intact and not contained:  # Intact avoids repeated termination commands
                        await terminate_balloon()
                        trigger = "Geofencing"
                        intact = False
            else:
                pass
        except Exception as e:
            print(f'\n{RED}{"Geolocation error:":<30}{RESET}{e}\n')
        await asyncio.sleep(10) # Wait 10 sec prior to next assessment


async def terminate_balloon():  
    global primary, intact
    try:
        print(f'{CYAN}{"Termination has been commanded."}{RESET}\n')
        if primary:
            timestamp = datetime.now().strftime("%H:%M:%S")
            servo.value = servo_open
            print(f'{MAGENTA}{"Servo opened":<25}{RESET}{timestamp}')  # Release the line
            await asyncio.sleep(3)
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f'{MAGENTA}{"Nichrome ON":<25}{RESET}{timestamp}') 
        heat_element.value = relay_on  
        await asyncio.sleep(heat_time)
        heat_element.value = relay_off  
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f'{MAGENTA}{"Nichrome OFF":<25}{RESET}{timestamp}\n') 
        print(f'{GREEN}{"Termination complete":<25}{RESET}\n') 
        if primary:
            await transmit_report()     # Send an update at termination
        else:
            pass
    except Exception as e:
            print(f'\n{RED}{"Termination error:":<25}{RESET}{e}\n')                       
                     

#-------------------- MAIN FUNCTION --------------------
async def main():
    task1 = asyncio.create_task(gps())
    task2 = asyncio.create_task(record())
    task3 = asyncio.create_task(display())
    task4 = asyncio.create_task(assess_airborne())
    if primary:
        task5 = asyncio.create_task(environmental_monitor())
        task6 = asyncio.create_task(electrical_monitor())
        #task7 = asyncio.create_task(baro_monitor())
        task8 = asyncio.create_task(periodic_update())
    task10 = asyncio.create_task(flight_timer(lambda: airborne))
    task11 = asyncio.create_task(geofencing())
    
    
    await task1
    await task2
    await task3
    await task4
    if primary:
        await task5
        await task6
        #await task7
        await task8
    await task10
    await task11
    
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\n', "User terminated program.")

