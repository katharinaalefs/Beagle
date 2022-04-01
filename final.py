import datetime as dt
from beagle_py import *

ch1_scalefactor = 4
ch2_scalefactor = 4
ch3_scalefactor = 4
ch4_scalefactor = 0

K_I_SCALE = 0.002756342
K_U_SCALE = 2048.0 / 2**31
K_P_SCALE = K_U_SCALE*K_I_SCALE*65536
K32_SPEED_2_RPM_SCALE = (30.0 / 2**20)
K16_SPEED_SCALE = (3.14159/16)
K32_SPEED_SCALE = (3.14159/2**20)
K_TORQUE_SCALE= K_I_SCALE*K_U_SCALE/ K16_SPEED_SCALE * 64

ch1_type = 'SIGNED_32'
ch2_type = 'SIGNED_32'
ch3_type = 'SIGNED_32'
ch4_type = 'SIGNED_32'

ch1_intscalefactor = 30.0/(2**21)
ch2_intscalefactor = K_TORQUE_SCALE
ch3_intscalefactor = (3.14159/2**20)
ch4_intscalefactor = 1

port       = 0      # open port 0 by default
megasamples = 50 # The sample rate in MHz
timeout    = 250    # in milliseconds
latency    = 200    # in milliseconds
length     = 0
num        = 0

mosi = []
devices = []

def detect():
    print("Detecting Beagle adapters...") 
    # Find all the attached devices
    (num, ports, unique_ids) = bg_find_devices_ext(2, 2)
    
    if num > 0:
        print ("%d device(s) found:" % num)
    
        # Print the information on each device
        for i in range(num):
            port      = ports[i]
            unique_id = unique_ids[i]
    
            # Determine if the device is in-use
            inuse = "(avail)"
            if (port & BG_PORT_NOT_FREE):
                inuse = "(in-use)"
                port  = port & ~BG_PORT_NOT_FREE
    
            # Display device port number, in-use status, and serial number
            print("    port = %d   %s  (%04d-%06d)" % \
                (port, inuse, unique_id / 1000000, unique_id % 1000000))
    else:
        print("No devices found.")

def gethandle():
    handle = bg_open(port)
    if (handle <= 0):
        print("Unable to open Beagle device on port %d" % port)
        print("Error code = %d" % handle)
        sys.exit()
    else:
        print("Connected to Beagle device on port %d" % port)
        return(handle)

def initialize(handle):
    # Disable the Beagle monitor
    bg_disable(handle)
    
    # Set the idle timeout.
    # The Beagle read functions will return in the specified time
    # if there is no data available on the bus.
    bg_timeout(handle, timeout)
    print("Idle timeout set to %d ms" %  timeout)
    
    # Configure the device for SPI
    bg_spi_configure(handle,
        BG_SPI_SS_ACTIVE_LOW,
        BG_SPI_SCK_SAMPLING_EDGE_FALLING,
        BG_SPI_BITORDER_MSB)
    bg_target_power(handle, BG_TARGET_POWER_OFF)
    
    # Set the sample rate in kilohertz (returns actual rate)
    actrate = bg_samplerate(handle, megasamples * 1000)
    print("Sample rate set to %d MHz" % int(actrate/1000))
    
    # Set the latency.
    # The latency parameter allows the programmer to balance the
    # tradeoff between host side buffering and the latency to
    # receive a packet when calling one of the Beagle read
    # functions.
    bg_latency(handle, latency)
    print("Latency set to %d ms" % latency)
    
    # Enable the Beagle monitor
    bg_enable(handle, BG_PROTOCOL_SPI)


def spidump (handle, max_bytes, num_packets):

    # Get the size of timing information for each transaction of size
    # max_bytes
    timing_size = bg_bit_timing_size(BG_PROTOCOL_SPI, max_bytes)

    # Get the current sampling rate
    samplerate_khz = bg_samplerate(handle, 0)

    # Start the capture
    if (bg_enable(handle, BG_PROTOCOL_SPI) != BG_OK):
        print("error: could not enable SPI capture; exiting...")
        sys.exit(1)

    print("index,time(ns),SPI,status,mosi0/miso0 ... mosiN/misoN")
    sys.stdout.flush()

    i = 0

    # Allocate the arrays to be passed into the read function
    data_mosi  = array_u08(max_bytes)
    data_miso  = array_u08(max_bytes)
    bit_timing = array_u32(timing_size)

    # Capture and print information for each transaction
    while (i < num_packets or num_packets == 0):

        # Read transaction with bit timing data
        (count, status, time_sop, time_duration, time_dataoffset, data_mosi, \
         data_miso, bit_timing) = \
         bg_spi_read_bit_timing(handle, data_mosi, data_miso, bit_timing)


        # Translate timestamp to ns
        time_sop_ns = TIMESTAMP_TO_NS(time_sop, samplerate_khz);

        sys.stdout.write( "%d,%u,SPI,(" % (i, time_sop_ns))

        if (count < 0):
            print("error=%d,", count)

        print_general_status(status)
        print_spi_status(status)
        sys.stdout.write( ")")

        # Check for errors
        i += 1
        if (count <= 0):
            print("")
            sys.stdout.flush()

            if (count < 0):
                break;

            # If zero data captured, continue
            continue;

        # Display the data

        for n in range(count):
            if (n != 0):         sys.stdout.write(", ")
            if ((n & 0xf) == 0): sys.stdout.write("\n    ")
            print("%02x/%02x" % (data_mosi[n], data_miso[n]),)
            #print(type(data_mosi[n]))
            #print('%02x' % data_mosi[n])
            mosi.append("%02x" % data_mosi[n])
            #print(type(mosi[n]))
        sys.stdout.write("\n")
        sys.stdout.flush()
        
        #print(mosi)

    
    # Stop the capture
    bg_disable(handle)

#==========================================================================
# UTILITY FUNCTIONS
#==========================================================================
def TIMESTAMP_TO_NS(stamp, samplerate_khz):
    return (stamp * 1000) // (samplerate_khz//1000)

def print_general_status (status):
    """ General status codes """
    print("",)
    if (status == BG_READ_OK) :
        print("OK",)

    if (status & BG_READ_TIMEOUT):
        print("TIMEOUT",)

    if (status & BG_READ_ERR_MIDDLE_OF_PACKET):
        print("MIDDLE",)

    if (status & BG_READ_ERR_SHORT_BUFFER):
        print("SHORT BUFFER",)

    if (status & BG_READ_ERR_PARTIAL_LAST_BYTE):
        print("PARTIAL_BYTE(bit %d)" % (status & 0xff),)


def print_spi_status (status):
    """No specific SPI status codes"""
    pass


def typeOffset(args):
    switcher = {
        'UNSIGNED_16': 0,
        'SIGNED_16'  : 0X8000,
        'UNSIGNED_32': 0,
        'SIGNED_32'  : 0x80000000,
        'UNSIGNED_16_OFFSET': 6554
    }
    return switcher.get(args, 0)
        
def typeShift(args):
    switcher = {
        'UNSIGNED_16' : 0,
        'SIGNED_16'   : 0,
        'UNSIGNED_16_OFFSET' : 0,
        'UNSIGNED_32' : 16,
        'SIGNED_32'   : 16
    }
    return switcher.get(args, 0)

detect()
handle = gethandle()
initialize(handle)
spidump(handle, 2, 5000)

def get_mosi_strings():

    #concatenate every two neighbors
    for i in range(0,len(mosi)-1,2):
        mosi[i] = mosi[i] + mosi[i+1]
    
    # remove remaining items from list 
    for i in range(len(mosi)-1,0,-2):
        mosi.pop(i)

    return mosi

print(get_mosi_strings())

channel1= []
channel2= []
channel3= []
channel4= []

for i in range(0, len(mosi)):
    if (mosi[i].startswith("3")):
        channel1.append(mosi[i])
    elif (mosi[i].startswith("7")):
        channel2.append(mosi[i])
    elif(mosi[i].startswith("b")):
        channel3.append(mosi[i])        
    elif (mosi[i].startswith("e")):
        channel4.append(mosi[i])

channels = {'channel1':[ch1_type, ch1_scalefactor , ch1_intscalefactor, channel1], 'channel2':[ch2_type, ch2_scalefactor, ch2_intscalefactor, channel2],\
            'channel3':[ch3_type, ch3_scalefactor, ch3_intscalefactor, channel3], 'channel4':[ch4_type, ch4_scalefactor, ch4_intscalefactor, channel4]}

def conversion(channel, ch_type, ch_scalefactor, ch_intscalefactor):
    for i in range(0, len(channel)):
        channel[i] = channel[i][1:]
        channel[i] = '0x' + channel[i]
        channel[i] = eval(channel[i])
        channel[i] = channel[i]<<4
        channel[i] = ((channel[i] << typeShift(ch_type)) - typeOffset(ch_type)) >> int(ch_scalefactor)
        channel[i] = int(channel[i])*ch_intscalefactor


for channel in channels:
    conversion(channels[channel][3], channels[channel][0], channels[channel][1], channels[channel][2])


print(channel3)