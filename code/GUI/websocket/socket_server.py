#!/usr/bin/env python3

import socket
import os
import csv
from datetime import datetime,timedelta
import sys
import time

#globala variabler
HOST = ''  # Standard loopback interface address (localhost)
PORT = 65430       # Port to listen on (non-privileged ports are > 1023)
CONNECTION_WATCHDOG = 1 #Timeout in seconds before a server-client connection
                        #is terminated.

#Directory for storage of data received from Raspberry Pi.
sensor_data_path = os.path.abspath('sensor_data/') + os.sep
#File with data to be sent to the robot.
#Commands and controlparameters.
command_file_path = os.path.abspath('./commands_to_transfer.txt')


#Datafield length for each datatype.
datalength = {
      1:1,
      2:1,
      3:1,
      4:1,
      5:1,
      6:2,
      7:2,
      8:2, 
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

#Template filenames for data storage.
file_name = {
      1:"Tejp_{}.csv",
      2:"Höger_fram_{}.csv",
      3:"Vänster_fram_{}.csv",
      4:"Höger_bak_{}.csv",
      5:"Vänster_bak_{}.csv",
      6:"Rakt_fram_{}.csv",
      7:"Vinkel_{}.csv",
      8:"Hjulsensor_{}.csv",  ##Har utgått
      9:"Start_lägesknapp_{}.csv",
      10:"Regl_kp_korridor_{}.csv",
      11:"Regl_ka_korridor_{}.csv",
      12:"Regl_kp_rotera_{}.csv",
      13:"Regl_ka_rotera_{}.csv",
      14:"Regl_kp_tejp_{}.csv",
      15:"Regl_ka_tejp_{}.csv",
      17:"GasHögerHjul_{}.csv",
      18:"GasVänster_{}.csv",
      19:"Styrbeslut_{}.csv",
      20:"KortasteVäg_{}.csv",
      21:"Karta_{}.csv",
      22:"Nödställd_rad_{}.csv",
      23:"Nödställd_kolumn_{}.csv",
      24:"RobotPos_rad_{}.csv",
      25:"RobotPos_kolumn_{}.csv",
      26:"Riktning_{}.csv"
    }


########################
#ska ta in en bytes och undersöka om vi har läst in hela medelandet
def data_transfer_fin(data_stream):
    data_tran_fin = False
    if len(data_stream) >= 2:
        data_length = int.from_bytes(data_stream[:2], byteorder='big', signed=False)
        if data_length + 2 >= len(data_stream):
            data_tran_fin = True


    return data_tran_fin


#Generate csv-formatted files from supplied data, based
#on its type.
def write_to_file(data_tag, data_info):
    global sensor_data_path
    #Time and date strings used for labeling filenames and rows within files.
    time_and_date_str = datetime.now().strftime("kl%H_dag%d_%h")
    time_str = datetime.now().strftime('%T')

    #Handle the case where data_info is and int.
    if type(data_info) is int:
        data_info = [data_info]


    #Directory path- and filename.
    file_path = sensor_data_path + file_name[data_tag].format(time_and_date_str)

    #Is file_path non-existent?
    is_new_file = not os.path.isfile(file_path)

    #Open file (or create if non-existent) related to a
    #certain data_tag.
    with open(file_path, "a", newline='') as file:
        csvwriter = csv.writer(file, dialect='excel')

        #Add column labels.
        if is_new_file:
            csvwriter.writerow(['Tid [h:m:s]', 'Värde'])

        #Append actual data (csv-formatted).
        csvwriter.writerow([time_str] + data_info)


##########################################################
#Saves bytesobject as bytefile. Takes a bytes object as input.
def save_map(data_info):
    global sensor_data_path
    time = datetime.now().strftime("kl%H_dag%d_%h")
    file = open(sensor_data_path + "Karta_{}".format(time),"wb")
    file.write(data_info)
    file.close()

#Uppdaterar kartan med ny information. Tar in ett bytes obejekt på tre bytes där
#första byten är raden, andra byten kolumnen och tredje byten är information.
def update_map(to_update):
    global sensor_data_path
    print('update_map: {}'.format(to_update))
    ROW_LEN = 101
    row = int.from_bytes(to_update[0:1], byteorder='big', signed=False)
    column = int.from_bytes(to_update[1:2], byteorder='big', signed=False)
    time = datetime.now().strftime("kl%H_dag%d_%h")
    map = b''

    try:
        file = open(sensor_data_path + file_name[21].format(time),"rb")
        map = file.read()
        file.close()
    except:
        print(sys.exc_info()[1])
        #Om det inte finns en nuvarande karta kolla att det inte finns en karta
        #från förra timmen
        try:
            last_hour = datetime.now() - timedelta(hours=1)
            file = open(sensor_data_path + file_name[21].format(last_hour.strftime("kl%H_dag%d_%h")),"rb")
            map = file.read()
            file.close()
        #Det fanns ingen nuvarande karta eller gammal karta skapa en ny med
        #endast outforskade rutor.
        except:
            map = b'\x00'*5151

    new_map = bytearray(map)
    new_map[ROW_LEN*row + column] = to_update[2]
    file = open(sensor_data_path + file_name[21].format(time),"wb")
    file.write(new_map)
    file.close()


#Splits data into constiuent data and saves in file.
#Takes a bytes object as input.
def split_data(data_stream):
    global datalength

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
                #minskar längden på listan så att nästa data_tag är första värdet
                write_to_file(data_tag,data_info)

            #Undersöker om det är ett av de data som är 1 byte lång
            elif datalength[data_tag] == 1:
                data_info = int.from_bytes(data_stream[1:2], byteorder='big', signed=False)
                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]
                #minskar längden på listan så att nästa data_tag är första värdet
                write_to_file(data_tag,data_info)

            #Undersöker om det är ett av de data som är 1 byte lång
            elif datalength[data_tag] == 2:
                data_info = int.from_bytes(data_stream[1:3], byteorder='big', signed=False)
                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]
                #minskar längden på listan så att nästa data_tag är första värdet
                write_to_file(data_tag,data_info)

            elif data_tag == 19:
                int_vector = []
                for vector_item in data_stream[1: 11]:
                    int_vector.append(vector_item)

                #funktionen som skriver datan till filen
                data_stream = data_stream[datalength[data_tag] + 1:]
                #minskar längden på listan så att nästa data_tag är första värdet
                write_to_file(data_tag,int_vector)

            elif data_tag == 20:
                int_vector = []
                length = int.from_bytes(data_stream[1:3], byteorder='big', signed=False)
                for byte in data_stream[3:3+length]:
                    int_vector.append(byte)

                #funktionen som skriver datan till filen
                data_stream = data_stream[length + 3:]
                #minskar längden på listan så att nästa data_tag är första värdet
                write_to_file(data_tag,int_vector)

            ##Sparar karta till fil.
            elif data_tag == 21:
                if len(data_stream) >= 5152:
                    save_map(data_stream[1:5152])
                data_stream = data_stream[datalength[data_tag] + 1:]

            #Uppdaterar kartan
            elif data_tag == 27:
                number_of_updates = data_stream[1]
                for index in range(number_of_updates):
                    update_now = data_stream[2 + index*3:index*3 + 5]
                    update_map(update_now)

                data_stream = data_stream[number_of_updates*3 + 2:]



########################################################

def intList_to_bytes(list_data: list) -> bytes:
    out_data = bytes(0)
    for x in list_data:
        out_data = out_data + x.to_bytes(1, 'big')
    return out_data

def read_file_commands():
    global command_file_path
    f_commands = open(command_file_path,"r")
    file_info = f_commands.readlines()
    f_commands.close()
    length = len(file_info)
    line_to_return = ""
    integers=[]

    if length != 0:
        i = 0

        while i < length:
            if file_info[i][0] != '#':
                break
            i += 1

        if i < length:
            for signs in file_info[i].split():
                integers.append( int(signs))
            f_commands = open("commands_to_transfer.txt","w")

            for line in file_info[i+1:]:
                f_commands.write(line)
            f_commands.close()

    if len(integers) == 0:
        integers.append(254)

    return intList_to_bytes(integers)


#Create a socket object to use as a server.
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    connection_no = 1

    #Listen for a connection and, once established,
    #exchange data with the client. When a transmission is
    #complete, and the connection is dropped; repeat. i. e.
    #listen for a new connection.
    while True:
        print('Listening on port {}.'.format(PORT))
        s.listen()
        conn, addr = s.accept()
        connection_start = time.time()
        print('Exchange no. {} started.'.format(connection_no))
        connection_no +=1
        with conn:
            print('Connected by', addr)

            #Fetch data from client.
            #Be aware, not all data may arrive in a single fetch.
            data_ = b''
            while True:
                data = conn.recv(1024)
                data_ = data_ + data

                #Terminate the fetch session if the time
                #past have exceeded the maximum allowed.
                if time.time()-connection_start >= CONNECTION_WATCHDOG:
                    break
                #This data stream (\x00\x00\xff) is used by the
                #client (Raspberry Pi) to test if the server responds.
                if data_ == b'\x00\x00\xff':
                    conn.sendall(b'\xfe')
                    break

                elif data_transfer_fin(data_):
                    break

            #Terminate the connection  (and listen for
            #a new) if the time past have exceeded the maximum allowed.
            if time.time()-connection_start >= CONNECTION_WATCHDOG:
                continue

            #Close the connection if it's only a server
            #response test.
            if data_ == b'\x00\x00\xff':
                print('Server test.')
                continue

            #Take care of data from client.
            try:
                split_data(data_)
            except:
                print(sys.exc_info()[1])

            #Send data to client.
            conn.sendall(read_file_commands())
            print('Exchange complete.')
