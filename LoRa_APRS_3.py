
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
SOURCE_SSID = '11'
DEST_ADDRESS = 'APRS  '
DEST_SSID = '0'
PATH_ADDRESS = 'WIDE2'
PATH_SSID = '2' 
FLAG = 0x7e   
CONTROL_FIELD = 0x03           
PROTOCOL_ID = 0xF0      # A PID value of 0xF0 is used to specify text content


def encode_address(address, ssid):
    """
    Encodes the source or destination address into the proper 7-byte format for AX.25 frames.
    Args:
        address (str): The callsign (e.g., 'KW5AUS').
        ssid (int or str): The SSID (e.g., '11') as a number or string.
    Returns:
        bytes: The encoded address in 7-byte format.
    """
    # Ensure SSID is a single byte (1 byte) by converting it to an integer
    if isinstance(ssid, str):
        ssid = int(ssid)  # Convert SSID string to an integer if it is a string
    # Concatenate callsign and SSID to form the full address
    full_address = address + chr(ssid)  # SSID as a single byte (chr() gives us the byte representation)
    # Ensure the address is exactly 7 bytes by padding with spaces if necessary
    padded_address = full_address.ljust(7)
    # Return the encoded result as bytes
    return padded_address.encode('utf-8')
    

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



def create_ax25_frame():    #<<<<< Check this is formatted properly with GPT <<<<<<<<
    """
    - This section creates the APRS message by combining the components into the format specified by the APRS protocol.
    - The APRS protocol can be found at: 
    """
    information_field = create_obj_report()  

    frame = bytearray()
    frame.extend(encode_address(DEST_ADDRESS,DEST_SSID))    # Destination
    frame.extend(encode_address(SOURCE_ADDRESS,SOURCE_SSID))    # Source
    frame.extend(encode_address(PATH_ADDRESS,PATH_SSID))    # Destination
    frame.append(CONTROL_FIELD)
    frame.append(PROTOCOL_ID)
    frame.extend(information_field.encode('utf-8'))
    
    print(f"{ORANGE}AX25 frame constructed{RESET}")
    print(f"{ORANGE}{'Output:':<15}{frame}{RESET}")  # Debug output
    print(f"{ORANGE}{'Data Type:':<15}{type(frame)}\n")   # Confirm it's a bytearray
    
    return frame

"""Calculate the Frame Check Sequence (FCS) for AX.25."""
def calculate_fcs(data):
    fcs = 0xFFFF
    for byte in data:
        fcs ^= byte  # XOR the byte into the FCS
        for _ in range(8):  # Process each bit
            if fcs & 0x0001:  # If the least significant bit is set
                fcs = (fcs >> 1) ^ 0x8408  # Shift and apply polynomial
            else:
                fcs >>= 1  # Just shift if the LSB is not set
                
    print(f"{YELLOW}FCS calculated{RESET}")
    print(f"{YELLOW}{'Output:':<15}{fcs}{RESET}")  # Debug output
    print(f"{YELLOW}{'Data Type:':<15}{type(fcs)}\n")   # Confirm it's a bytearray
    
    return ~fcs & 0xFFFF  # Invert and mask to 16 bits
    # The output is a 16-bit unsigned integer (int) but must be in a byte format, which will be converted later
    

async def create_aprs_message():
    frame = create_ax25_frame()       # Create the AX.25 frame
    fcs = calculate_fcs(frame)      # Calculate the FCS
    fcs_bytes = fcs.to_bytes(2, byteorder="little")  # Convert FCS to bytes. 2 bytes, little-endian format

    print(f"{GREEN}Starting APRS message{RESET}")
    print(f"{GREEN}{'Output:':<15}{fcs_bytes}{RESET}")  # Debug output
    print(f"{GREEN}{'Data Type:':<15}{type(fcs_bytes)}{RESET}\n")   # Confirm it's a bytearray
    
    message = bytearray()    
    message.append(FLAG)
    message.append(FLAG)
    message.append(FLAG)
    message.extend(frame)
    message.extend(fcs_bytes)
    message.append(FLAG)
    
    print(f"APRS Message (bytearray): {message}")  # Debug output
    print(f"APRS Message Type: {type(message)}")   # Confirm it's a bytearray
    
    return message



async def transmit_report():  
    """
    This section takes the UI framte byte array and then transmits the message
    - The purpose for this formating is to integrate with APRS. 
    """  
    global tx_counter, msg_sent, object_report, tx_time, tx_rate
    try:
        byte_message = await create_aprs_message()     # Message formatted as byte array
        message_list = list(byte_message)                   # Converts the byte array to a list of integers because LoRa.write() expects a list of integers
        
        print(f"{MAGENTA}Starting transmission{RESET}\n")
        print(f"{MAGENTA}{'Output:':<15}{message_list}{RESET}")  # Debug output
        print(f"{MAGENTA}{'Data Type:':<15}{type(message_list)}{RESET}\n")   # Confirm it's a bytearray
        
        print(f"Byte message type: {type(byte_message)}")  # Should be bytearray
        print(f"Message list type: {type(message_list)}")  # Should be list
        print(f"Message list contents: {message_list}")    # Ensure all elements are integers

        
        LoRa.beginPacket()
        LoRa.write(message_list, len(message_list))     # This sends the message, which is now a list of integers
        LoRa.write([tx_counter], 1)     # This sends the counter value, which is likely an additional byte appended to the message, perhaps to indicate a message sequence number or packet identifier.
        LoRa.endPacket()
        LoRa.wait()
        
        if not all(isinstance(i, int) for i in message_list):
            raise ValueError("Message list contains non-integer elements")

        
        tx_counter = (tx_counter + 1) % 256
        msg_sent = datetime.now().strftime("%H:%M:%S")
        tx_time = f"{LoRa.transmitTime() / 1000:.2f}"
        tx_rate = f"{LoRa.dataRate():.2f}"
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

