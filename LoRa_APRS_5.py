
import asyncio
from datetime import datetime, date
from LoRaRF import SX127x
import ax25


(RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, MAGENTA, RESET) = ('\033[91m', '\033[38;5;208m', '\033[93m', '\033[92m', '\033[96m', '\033[94m', '\033[95m', '\033[0m')

computer_type = "A"
balloon_id = "99c"
gps_lat = 38.3936
gps_lon = -86.5952
gps_spd = 69
gps_trk = 101
gps_day = '18'
gps_alt = 5555
gps_time = '20:05:59'

intact = True
flight_time = 600
trigger = "None"

update_interval = 2         # Minutes between LoRa update transmissions
msg_iterations = 2          # Total number of times the message will be sent per update burst
msg_interval = 5            # Number of seconds between the burst messages (>=5 required)

tx_counter = 0

testing_mode = True


#-------------------- SX1278 Initialization --------------------

LoRa = SX127x()
if computer_type in ["A", "C"]:
    """
    - The SX1278 LoRa transceiver is how we broadcast messages over 433 MHz to the ground station(s). 
    - This code will setup the transceiver for operation with the designated parameters. 
    - LoRa APRS operates at 433.775 MHz with 125 MHz bandwidth, 12 spreading factor, and 5 code rate
    - Ensure the base station is operating with the same settings
    - The B computers are not equipped with transmitters
    """
    # SPI Port Configuration: bus id 0 and cs id 1 and speed 7.8 Mhz
    LoRa.setSpi(0, 0, 7800000)  
    # I/O Pins Configuration: set RESET->22, BUSY->23, DIO1->26, TXEN->5, RXEN->25
    LoRa.setPins(22, 23)  
    LoRa.begin()
    # Modem Configuration 
    LoRa.setTxPower(17, LoRa.TX_POWER_PA_BOOST)     # Set TX power +17 dBm using PA boost pin
    LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING, LoRa.RX_GAIN_AUTO)    # AGC on, Power saving gain
    LoRa.setFrequency(433775000)                    # Set frequency to 433.775 Mhz, the LoRa APRS frequency, or 433.5
    # Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
    LoRa.setSpreadingFactor(12)     # 12 (max) Prioritizes long-range communication and can tolerate a slower data rate
    LoRa.setBandwidth(125000)       # 125 kHz: most commonly used bandwidth for LoRa and is ideal for long-range communication
    LoRa.setCodeRate(5)   # 8 is best for very noisy or long-range applications where maximum reliability is necessary    
    # Packet configuration 
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 15, True, False)   # set explicit header mode, preamble length 12, payload length 15, CRC on and no invert IQ operation
    # Set syncronize word for public network (0x3444)
    LoRa.setSyncWord(0x3444)    # Set syncronize word for public network (0x3444)

    print(f"\n{CYAN}LoRa Transceiver configured.{RESET}\n")

# ---------- COMMUNICATIONS ---------- 
SOURCE_ADDRESS = 'KW5AUS'
SOURCE_SSID = 11 # should be 11
DEST_ADDRESS = 'APRS'     
DEST_SSID = 0
PATH_ADDRESS = 'WIDE2'
PATH_SSID = 2
FLAG = 0x7e   
CONTROL_FIELD = 0x03           
PROTOCOL_ID = 0xF0      # A PID value of 0xF0 is used to specify text content
  

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


def create_obj_report():
    """
    This section creates & formats the 'Object Report' for placement in the Information Field of the AX.25 message.
    """  
    global object_report
    
    info_field_data_id = ";"                # The ; is the APRS Data Type Identifier for an Object Report
    object_name = "BALON_" + balloon_id     # Fixed 9-character Object name, which may consist of any printable ASCII characters
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
    print(f"{RED}Object report constructed{RESET}")
    print(f"{RED}{'Output:':<15}{object_report}{RESET}")  # Debug output
    print(f"{RED}{'Data Type:':<15}{type(object_report)}\n")   # Confirm it's a bytearray
    
    return object_report        


def create_LoRa_frame():    #<<<<< Check this is formatted properly with GPT <<<<<<<<
    """
    The LoRa frame for LoRa-APRS is simply:

    The bytes 0x3C, 0xFF, 0x01
    The "TNC2" text representation of the APRS message (the same format that APRS-IS would use), e.g. the text KC2G>APRS,WIDE2-2:!4101.43NI07408.26W#
    """
    information_field = create_obj_report()  

    lora_frame = (
        f"{SOURCE_ADDRESS}-{SOURCE_SSID}>{DEST_ADDRESS},{PATH_ADDRESS}-{PATH_SSID}!{'Hello World'}"
    )

    print(f"{ORANGE}LoRa frame constructed{RESET}")
    print(f"{ORANGE}{'Output:':<15}{lora_frame}{RESET}")  # Debug output
    
    byteframe = bytearray()
    # Header
    #byteframe.append(0x3C)  
    #byteframe.append(0xFF)      # Appends the hexadecimal value 0xFF as a byte
    #byteframe.append(0x01)      # Appends the hexadecimal value 0x01 as a byte
    # APRS Data
    byteframe.extend(lora_frame.encode('utf-8'))    # Encodes the string into bytes and extends the bytearray
    
    print(f"{ORANGE}{'Byte Output:':<15}{byteframe}{RESET}\n")
    
    return byteframe


async def transmit_report():  
    """
    This section takes the UI framte byte array and then transmits the message
    - The purpose for this formating is to integrate with APRS. If we are unable to get the message to upload to the APRS network,
    we can use a simpler message format
    """  
    global tx_counter, msg_sent, tx_time, tx_rate
    try:
        byteframe = create_LoRa_frame()     # Message formatted as byte array
        message_list = list(byteframe)      # Converts the byte array to a list of integers because LoRa.write() expects a list of integers
        
        LoRa.beginPacket()
        LoRa.write(0x3C)
        LoRa.write(0xFF)
        LoRa.write(0x01)
        LoRa.write(message_list, len(message_list))     # This sends the message, which is now a list of integers
        LoRa.write([tx_counter], 1)     # This sends the counter value, which is likely an additional byte appended to the message, perhaps to indicate a message sequence number or packet identifier.
        LoRa.endPacket()
        LoRa.wait()

        tx_counter = (tx_counter + 1) % 256
        msg_sent = datetime.now().strftime("%H:%M:%S")
        tx_time = f"{LoRa.transmitTime() / 1000:.2f}"
        tx_rate = f"{LoRa.dataRate():.2f}"
        
        print(f"{YELLOW}{'Output:':<15}{message_list}{RESET}")  # Debug output
        print(f"{YELLOW}{'Data Type:':<15}{type(message_list)}{RESET}") 
        #message_string = bytes(message_list).decode('utf-8') 
        #print(f"{YELLOW}{'Data Type:':<15}{type(message_string)}{RESET}/n") 
        
        if not all(isinstance(i, int) for i in message_list):
            raise ValueError("Message list contains non-integer elements") 
        
    except Exception as e:
        print(f"\n{RED}{'Transmit Error:':<25}{RESET}{e}\n")
  
        
async def periodic_update():  #~~~~~ TASK 9 ~~~~~
    """
    - This code sends the position update with 'update_interval'*60 minutes between messages. 
    - Needs: update to deconflict the balloon transmissions based on the time of day so two balloons 
    are not transmitting over each other and blocking the signal.
    - Only the A and C computers are equipped with transmitters
    """   
    while computer_type in ["A", "C"]:
        for _ in range(msg_iterations):  # Loop for a fixed number of iterations
            await transmit_report()  # Call the async function
            await asyncio.sleep(msg_interval)  
        if testing_mode == True:
            print("----------------------------------------------------------------------------------------------")
            print(f"{MAGENTA}{'Messages sent:':<25}{CYAN}{msg_iterations}{RESET} on {LoRa._frequency / 1000000:.3f} MHz")
            print(f"{BLUE}{'Transmit time:':<25}{RESET}{LoRa.transmitTime() / 1000:0.2f}{' s'}")
            print(f"{BLUE}{'Data rate:':<25}{RESET}{LoRa.dataRate():0.2f}{' byte/s'}")
            print("----------------------------------------------------------------------------------------------")
        await asyncio.sleep(update_interval * 60)  # Wait some fixed number of minutes before the next update
  
       
#-------------------- MAIN FUNCTION --------------------
async def main():
    task9 = asyncio.create_task(periodic_update()) 
    
    await task9

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\n', "User terminated program.")

