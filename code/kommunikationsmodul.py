#!/usr/bin/env python3
import socket
import time
import spidev
import sys
import math

###########################
#   Global containers
###########################
data_to_wifi = bytes(0)
data_from_wifi = bytes(0)
PC_IP = '192.168.4.18'   #IP address for the server.
PC_PORT = 65432   #Port for the server.
SPI_SPEED = 32768
SPI_BUFF_SIZE = 4000 #This buffer size limit (in bytes) may
                     #ideally be less than the maximum buffer size allowed by spidev.

SPI_MAX_SIZE = 10000 #Maximum data transfer size (in bytes) allowed. Larger
                     #SPI transfers will be aborted.

SLEEP_TIME = 0.5     #

#Datafield length for each datatype.
datalength = {
      1:1,
      2:1,
      3:1,
      4:1,
      5:1,
      6:2,
      7:2,
      9:1,
      10:1,
      11:1,
      12:1,
      13:2,
      14:1,
      15:2,
      16:1,
      17:1,
      18:1,
      19:10,
      20:45,
      21:5151,
      22:1,
      23:1,
      24:1,
      25:1,
      26:1,
      27:45
    }

###########################
#   Application specific errors
###########################
class FailedToConnectWithServer( Exception ): pass
class BadPackageFormat( Exception ): pass
class FailedToSendToServer( Exception ): pass
class FailedToFetchFromServer( Exception ): pass

###########################
#   Procedures
###########################
#Splits data into constiuent data and saves in file.
#Takes a bytes object as input.
def split_data(data_stream):
    global datalength, SLEEP_TIME

    if len(data_stream) >= 2:
        data_stream = data_stream[2:]

        while len(data_stream) > 0:
            data_tag = data_stream[0]  #Detta är datans nummer
            data_info = 0

            #Hanterar stoppvilkoret
            if data_tag == 254:
                data_stream = []

            elif (data_tag == 1):
                data_info = int.from_bytes(data_stream[1:2], byteorder='big', signed=True)
                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]

            #Switch status
            elif data_tag == 9:
                #Remove delay between SPI transfers in manual mode.
                if data_stream[1] == 0:
                    #print('Manual speed set.') #!!!!!!!!!!!!!!!!!!
                    SLEEP_TIME = 0
                elif (data_stream[1] == 1) or (data_stream[1] == 2):
                    SLEEP_TIME = 0.5
                return

            #Undersöker om det är ett av de data som är 1 byte lång
            elif datalength[data_tag] == 1:
                data_info = int.from_bytes(data_stream[1:2], byteorder='big', signed=False)
                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]

            #Undersöker om det är ett av de data som är 1 byte lång
            elif datalength[data_tag] == 2:
                data_info = int.from_bytes(data_stream[1:3], byteorder='big', signed=False)
                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]

            elif data_tag == 19:
                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]

            elif data_tag == 20:
                length = int.from_bytes(data_stream[1:3], byteorder='big', signed=False)
                #funktionen som skriver datan till filen
                data_stream = data_stream[length + 3:]

            ##Sparar karta till fil.
            elif data_tag == 21:
                data_stream = data_stream[datalength[data_tag] + 1:]

            #Uppdaterar kartan
            elif data_tag == 27:
                number_of_updates = data_stream[1]
                data_stream = data_stream[number_of_updates*3 + 2:]

#Bolean procedure which will determine whether data_stream, which is the
#data received from the server, is complete or not.
#data_stream may contain either one of the datapackages (with ID):
#10, 11, 12 , 13, 14, 15, 16 and 254.
def data_transfer_fin(data_stream: bytes) -> bool:
    #Extract the frontmost ID tag contained in data_steram.
    ID_tag = data_stream[0]

    #Identify and acknowledge a server response test.
    if (ID_tag == 254):
        #Check if the package data-field
        #is 0 bytes wide (i. e. data_stream is 1 bytes wide)
        if len(data_stream) == 1:
            return True
        elif len(data_stream) > 1:
            raise BadPackageFormat('Received datapackage (ID: {}) violates its specifiation.'.format(ID_tag))
        else:
            return False #Report data_stream as incomplete.


    elif (ID_tag == 16):
        #Check if the package data-field
        #is 1 bytes wide (i. e. data_stream is 2 bytes wide)
        if len(data_stream) == 2:
            return True
        elif len(data_stream) > 2:
            raise BadPackageFormat('Received datapackage (ID: {}) violates its specifiation.'.format(ID_tag))
        else:
            return False #Report data_stream as incomplete.


    #Check for control parameters.
    elif (ID_tag == 10 or ID_tag == 11 or ID_tag == 12  or ID_tag == 14):
        #Check if the package data-field
        #is 1 byte wide (i. e. data_stream is 2 bytes wide)
        if len(data_stream) == 2:
            return True
        elif len(data_stream) > 2:
            raise BadPackageFormat('Received datapackage (ID: {}) violates its specifiation.'.format(ID_tag))
        else:
            return False #Report data_stream as incomplete.


    #Check for long control parmeters.
    elif (ID_tag == 13  or ID_tag == 15):
        #Check if the package data-field
        #is 2 byte wide (i. e. data_stream is 3 bytes wide)
        if len(data_stream) == 3:
            return True
        elif len(data_stream) > 3:
            raise BadPackageFormat('Received datapackage (ID: {}) violates its specifiation.'.format(ID_tag))
        else:
            return False #Report data_stream as incomplete.

    else:
        raise BadPackageFormat('Received datapackage (ID: {}) doesn\'t match any which was expected.'.format(ID_tag))

#This procedure takes a list of ints and
#returns a bytes object (immutable array).
def intList_to_bytes(list_data: list) -> bytes:
    out_data = bytes(0)
    for x in list_data:
        out_data = out_data + x.to_bytes(1, 'big')
    return out_data

#This procedure takes a bytes object and
#returns a list of integers.
def bytes_to_intList(in_data: bytes) -> list:
    out_data = []
    for x in in_data:
        out_data.append(x)
    return out_data

#This procedure takes a bytes object as argument.
#Initially, the procedure tries to establish a connection to the server (pc),
#and, if it succeeds, will then proceed to send data to pc.
#After transmission of this data, the procedure will continue by fetching data supplied
#by the server and return this data as a bytes object as well.
def wifi_exchange(out_data_: bytes) -> bytes:
    #Client-specific constants
    global PC_IP, PC_PORT

    #Create a socket (network interface) object to use as a client.
    pc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #Connect to server.
    try:
        pc.connect((PC_IP, PC_PORT))
    except:
        raise FailedToConnectWithServer('Failed to establish connection with server on\n'
                         +'\tAddress: {}\n'.format(PC_IP) + '\tPort: {}\n'.format(PC_PORT))

    #Send data to server.
    try:
        pc.sendall(out_data_)
    except:
        pc.close()
        raise FailedToSendToServer('Failed to send data to server over wifi.')

    #Fetch data from server.
    #Be aware, not all data may arrive in a single fetch.
    in_data_ = bytes(0)
    while True:
        try:
            in_data = pc.recv(1024)
        except:
            pc.close()
            raise FailedToFetchFromServer('Failed to fetch data from server over wifi.')
        in_data_ = in_data_ + in_data
        try:
            is_fetch_complete = data_transfer_fin(in_data_)
        except:
            #Reraise the exception to let the caller of wifi_exchange
            #handle the error.
            raise sys.exc_info()[1]

        if is_fetch_complete:
            break

        if not in_data:
            break

    #Identify and acknowledge a server response test.
    if in_data_ == b'\xfe':
        in_data_ = b''

    pc.close()
    return in_data_


def exchange_data():

    global data_to_wifi, data_from_wifi, SPI_SPEED, SPI_BUFF_SIZE, SPI_MAX_SIZE
    ###########################
    #   SPI exchanges
    ###########################
    wifi_length_used = False

    spi = spidev.SpiDev(0, 1)
    spi.max_speed_hz = SPI_SPEED

    #The first package to send over SPI,
    #before transmission of data.
    spi_start_message = [253, 0, 254]

    #print('Start message sent: {}'.format(spi_start_message)) #!!!!!!!!!!!!!!!!!
    print('Start message sent: {}'.format(spi_start_message)) #!!!!!!!!!!!!!!!!!!!!!!
    #Exchange of start-messages between the modules.
    try:
        spi.xfer2(spi_start_message)
    except:
        print(sys.exc_info()[1])
        spi.close()
        return

    print('Start message received: {}'.format(spi_start_message)) #!!!!!!!!!!!!!!!!!

    if spi_start_message[1] == 253:
        print('Terminate 253')
        spi.close()
        return


    #Extract length of data to recieve.
    data_length = int.from_bytes(intList_to_bytes(spi_start_message[1:]), byteorder='big', signed=False)

    #print('Extracted datalenght: {}'.format(data_length)) #!!!!!!!!!!!!!!!!!!!!
    if (data_length +1) > SPI_MAX_SIZE:
        print('Control module requests to send {} bytes of data,\n'.format(data_length)
                +'but SPI_MAX_SIZE is currently set to {} bytes.'.format(SPI_MAX_SIZE))
        data_length = SPI_MAX_SIZE

    #Assemble load to send over SPI.
    data_to_spi = bytes_to_intList(data_from_wifi)
    data_to_spi = [255] + data_to_spi + [254]

    #print('Data to send: {}'.format(data_to_spi)) #!!!!!!!!!!!!!!!!!!!!!!!!!

    if (data_length +1) > len(data_to_spi):
        data_to_spi += [0]*(data_length +1 -len(data_to_spi))
    else:
        wifi_length_used = True

    print('Data to send: {}'.format(data_to_spi)) #!!!!!!!!!!!!!!!!!!!!!!!!!

    #Exchange of actual data.
    #Perform exchanges in smaller chunks if data surpasses SPI_BUFF_SIZE.
    while True:
        exchange_no = 0
        data = []

        if (data_length +1) > SPI_BUFF_SIZE:
            data = data_to_spi[0:SPI_BUFF_SIZE]
        else:
            data = data_to_spi


        while (data_length +1) > SPI_BUFF_SIZE:
            try:
                spi.xfer2(data)
            except:
                print(sys.exc_info()[1])
                spi.close()
                return
            data_to_spi[(exchange_no*SPI_BUFF_SIZE) : ((exchange_no +1)*SPI_BUFF_SIZE)] = data
            exchange_no += 1
            data_length -= SPI_BUFF_SIZE
            data = data_to_spi[(exchange_no*SPI_BUFF_SIZE) : ((exchange_no +1)*SPI_BUFF_SIZE)]

        data = data_to_spi[(exchange_no*SPI_BUFF_SIZE):]
        try:
            spi.xfer2(data)
        except:
            print(sys.exc_info()[1])
            spi.close()
            return

        data_to_spi[(exchange_no*SPI_BUFF_SIZE):(exchange_no*SPI_BUFF_SIZE +len(data))] = data


        if wifi_length_used:
            data_to_spi = data_to_spi[:data_length +1]

        if data_to_spi[1] != 255:
            print('Exchange completed 255') #!!!!!!!!!!
            break

    spi.close()

    print('Data recieved: {}'.format(data_to_spi)) #!!!!!!!!!!!!!!!!!!!!!!
    data_to_spi = [math.floor(data_length/256)] + [data_length % 256] + data_to_spi[1:] #Append data length to send.
    data_to_wifi = intList_to_bytes(data_to_spi)

    split_data(data_to_wifi)

    ##############################
    #   WiFi exchanges
    ##############################
    #Start data exchange between the PC and this device.
    try:
        data_from_wifi = wifi_exchange(data_to_wifi)

    #Further data exchanges over wifi shouldn't be
    #stalled if a corrupt package were fetched from the server.
    except BadPackageFormat as e:
        print(e)
    except:
        raise sys.exc_info()[1]

#This procedure runs the main loop of the script.
#It begins by trying to connect to a server connected on the address and
#port-number defined in the header of this file. If this attempt is succeesfull,
#the procedure will proceed with the exchange of data between SPI and Wi-Fi.
def main():
    global SLEEP_TIME
    while True:

        #Test connection with server before proceeding.
        #Wait 3 seconds between retries upon failure.
        try:
            wifi_exchange(b'\x00\x00\xff')
        except:
            print(sys.exc_info()[1])
            time.sleep(3)
            continue

        #Start a data exchange session between SPI and wifi.
        while True:
            try:
                exchange_data()
            except:
                print(sys.exc_info()[1])
                break

            #Time to wait between exchanges.
            time.sleep(SLEEP_TIME)

main()
