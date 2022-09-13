#!/usr/bin/env python3
from tkinter import *
from tkinter import ttk
import csv
from datetime import datetime
import os
from gui import Stack
from PIL import ImageTk, Image
import sys

#Directory for storage of data received from Raspberry Pi.
sensor_data_path = os.path.abspath('websocket/sensor_data/') + os.sep
#File with data to be sent to the robot.
#Commands and controlparameters.
command_file_path = os.path.abspath('websocket/commands_to_transfer.txt')
file_name = {
      1:"Tejp_{}.csv",
      2:"Höger_fram_{}.csv",
      3:"Vänster_fram_{}.csv",
      4:"Höger_bak_{}.csv",
      5:"Vänster_bak_{}.csv",
      6:"Rakt_fram_{}.csv",
      7:"Vinkel_{}.csv",
      8:"Hjulsensor_{}.csv",
      9:"Start_lägesknapp_{}.csv",
      10:"Regl_kp_korridor_{}.csv",
      11:"Regl_ka_korridor_{}.csv",
      12:"Regl_kp_rotera_{}.csv",
      13:"Regl_ka_rotera_{}.csv",
      14:"Regl_kp_tejp_{}.csv",
      15:"Regl_ka_tejp_{}.csv", ###Observera att data 16 ej inte finns
      17:"GasHögerHjul_{}.csv",
      18:"GasVänster_{}.csv",
      19:"Styrbeslut_{}.csv",
      20:"KortasteVäg_{}.csv",    ##Denna är variabel längd
      21:"Karta_{}.csv",
      22:"Nödställd_rad_{}.csv",
      23:"Nödställd_kolumn_{}.csv",
      24:"RobotPos_rad_{}.csv",
      25:"RobotPos_kolumn_{}.csv",
      26:"Riktning_{}.csv"
    }

window = Tk()
window.title("Datormodul")



#This small interface is used to send control commands
#to the robot in manual mode.
class SteeringCtl():
    __pressed_keys = Stack()
    __command_aliases = {
                        'w':'forward_drive',
                        'a':'rotate_left',
                        'd':'rotate_right',
                        's':'reverse_drive',
                        'q':'forward_left',
                        'e':'forward_right'}
    __command_widgets = {}

    def __init__(self, widgets):
        self.__command_widgets = widgets

    #Callback method used to send control commands to the robot
    #when certain keyboard-keys are pressed.
    def key_pressed(self, event):
        key = event.keysym
        if key == 'g':
            save_to_file('16 6')
        elif key == 'r':
            save_to_file('16 5')
        elif (not self.__pressed_keys.is_in(key)) and (key in self.__command_aliases):
            self.__pressed_keys.push(event.keysym)
            self.action(self.__command_aliases[event.keysym])


    ##Callback method used to send control commands to the robot
    #when certain keyboard-keys are released.
    def key_released(self, event):
        key = event.keysym
        if key == 'r':
            save_to_file('16 0')
        elif key == 'g':
            save_to_file('16 0')
        elif self.__pressed_keys.is_in(key):
            self.__pressed_keys.remove(key)
            if self.__pressed_keys.size() == 0:
                self.action('break')
            else:
                self.action(self.__command_aliases[self.__pressed_keys.top()])


    def action(self, command):

        for label in self.__command_widgets.values():
            label.off()

        if command in self.__command_widgets:
            self.__command_widgets[command].on()

        if command == 'forward_drive':
            save_to_file('16 1')
        elif command == 'reverse_drive':
            save_to_file('16 2')
        elif command == 'rotate_right':
            save_to_file('16 3')
        elif command == 'rotate_left':
            save_to_file('16 4')
        elif command == 'forward_left':
            save_to_file('16 8')
        elif command == 'forward_right':
            save_to_file('16 7')
        elif command == 'break':
            save_to_file('16 0')

class SteeringLabel():
    __label = None
    __on_icon = None
    __off_icon = None

    def __init__(self, label, on_icon, off_icon):
        self.__label = label
        self.__on_icon = on_icon
        self.__off_icon = off_icon

    def on(self):
        self.__label['image'] = self.__on_icon

    def off(self):
        self.__label['image'] = self.__off_icon


#Callback function triggered upon closing of the window.
def on_closing(_event):
    #Restore (enable) repeated keyboard key before exit.
    os.system('xset r on')




#######################################################################
##          FUNCTIONS
######################################################################


####################### För regulator parametrarna ######################
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


def save_to_file(to_send):
    print(to_send)
    global command_file_path
    file = open(command_file_path,"a")
    file.write(to_send + "\n" )
    file.close()

#Spara dreglerparameter som skrivs i GUI till filen som servern läser.
#Sparar ocskå i csv-filen.
def send_regler_par():
    to_send_list = [ent_kp_korr, ent_ka_korr, ent_kp_rot, ent_ka_rot, ent_kp_tejp, ent_ka_tejp]
    to_send = ""
    for i in range(6):
        to_send += "{} ".format(10 + i) + to_send_list[i].get()
        if i != 6:
            to_send += "\n"
        write_to_file(10 + i , int(to_send_list[i].get()))
    save_to_file(to_send)

#hämtar Reglerparametrar från fil för att visa i GUI.
def get_regler_par():
    global sensor_data_path
    to_get = [ent_kp_korr, ent_ka_korr, ent_kp_rot, ent_ka_rot, ent_kp_tejp, ent_ka_tejp]
    for i in range(6):
        time_and_date_str = datetime.now().strftime("kl%H_dag%d_%h")
        file_path = sensor_data_path + file_name[10 + i].format(time_and_date_str)

        with open(file_path) as csvfile:
            regulator_par = csv.reader(csvfile,delimiter = ' ', quotechar='|')
            last_row = ""
            for row in regulator_par:
                last_row = row
            to_get[i].delete(0,END)
            to_get[i].insert(0,last_row[0].split(',')[1])



################################### För kördata #########################

def update_drive_data():
    global sensor_data_path

    tape_dist = {
        4:"4 cm",
        45:"3 cm",
        5:"2 cm",
        56:"1 cm",
        6:"0 cm",
        67:"-1 cm",
        7:"-2 cm",
        78:"-3 cm",
        8:"-4 cm",
        31:"STOP"

    }
    print("inne i updatera kördata")
    update_list = [[1,lbl_tape_data,""],[2,lbl_dist_right_forw_data," cm"],[3,lbl_dist_left_forw_data," cm"],
                   [4,lbl_dist_right_back_data," cm"],[5,lbl_dist_left_back_data," cm"],[6,lbl_dist_forw_data," cm"],
                   [7,lbl_angle_data," grader"],[9,lbl_breaker_data,""],[17,lbl_right_speed_data," %"],
                   [18,lbl_left_speed_data," %"],[8,lbl_dist_wheel_data, " cm"]]

    for tripple in update_list:
        time_and_date_str = datetime.now().strftime("kl%H_dag%d_%h")
        file_path = sensor_data_path + file_name[tripple[0]].format(time_and_date_str)
        try:
            with open(file_path) as csvfile:
                regulator_par = csv.reader(csvfile,delimiter = ' ', quotechar='|')
                last_row = ""
                for row in regulator_par:
                    last_row = row
                if tripple[0] == 1:
                    tape_data = int(last_row[0].split(',')[1])
                    print("tejp rådata: {}".format(tape_data))
                    tripple[1].config(text = tape_dist[tape_data]  + tripple[2])
                else:
                    tripple[1].config(text = last_row[0].split(',')[1]  + tripple[2])
        except:
            print(sys.exc_info()[1])
            print("filerna finns ej {}".format(file_path))
    #window.after(1000,update_drive_data)





#################################################################
#      REGLER Pararmetrar skicka ta emot frame (2,1)
################################################################
frm_send = ttk.Frame(master=window)

frm_send.grid(row=2,column=0)

lbl_text = ttk.Label(master = frm_send,text="Reglerparametrar")
lbl_text.grid(row=0,column=2)

lbl_kp_korr =ttk.Label(master=frm_send,text="kp korridor:")
lbl_kp_korr.grid(row=1,column=0)
ent_kp_korr = ttk.Entry(master = frm_send)
ent_kp_korr.grid(row=1,column=1)
lbl_ka_korr =ttk.Label(master=frm_send,text="ka korridor:")
lbl_ka_korr.grid(row=2,column=0)
ent_ka_korr = ttk.Entry(master = frm_send)
ent_ka_korr.grid(row=2,column=1)

lbl_kp_rot =ttk.Label(master=frm_send,text="kp rotation:")
lbl_kp_rot.grid(row=3,column=0)
ent_kp_rot = ttk.Entry(master = frm_send)
ent_kp_rot.grid(row=3,column=1)
lbl_ka_rot =ttk.Label(master=frm_send,text="ka rotation:")
lbl_ka_rot.grid(row=3,column=2,sticky="E")
ent_ka_rot = ttk.Entry(master = frm_send)
ent_ka_rot.grid(row=3,column=3)


lbl_kp_tejp =ttk.Label(master=frm_send,text="kp tejp:")
lbl_kp_tejp.grid(row=1,column=2,sticky="E")
ent_kp_tejp = ttk.Entry(master = frm_send)
ent_kp_tejp.grid(row=1,column=3)
lbl_ka_tejp =ttk.Label(master=frm_send,text="ka tejp:")
lbl_ka_tejp.grid(row=2,column=2,sticky="E")
ent_ka_tejp = ttk.Entry(master = frm_send)
ent_ka_tejp.grid(row=2,column=3)

btn_send = ttk.Button(master = frm_send,text="skicka",command=send_regler_par)
btn_send.grid(row=4,column=3)
btn_get = ttk.Button(master = frm_send,text="hämta",command=get_regler_par)
btn_get.grid(row=4,column=1)


##################################################################################
##          Visa nuvarande parameterar frame (1,2)
##############################################################################
frm_show_data = ttk.Frame(master=window)
frm_show_data.grid(row=2,column=1,columnspan = 2)

lbl_drive_data = ttk.Label(master=frm_show_data,text="KÖRDATA")
lbl_drive_data.grid(row=0,column=1)
############# Etiketter för datat
##AVSTÅND
lbl_dist_forw = ttk.Label(master=frm_show_data,text="Avstånd framåt: ")
lbl_dist_forw.grid(row=1,column=0,sticky = "E")
#höger
lbl_dist_right_forw = ttk.Label(master=frm_show_data,text="Avstånd främre höger: ")
lbl_dist_right_forw.grid(row=2,column=2,sticky = "E")
lbl_dist_right_back = ttk.Label(master=frm_show_data,text="Avstånd bakre höger: ")
lbl_dist_right_back.grid(row=3,column=2,sticky = "E")
#vänster
lbl_dist_left_forw = ttk.Label(master=frm_show_data,text="Avstånd främre vänster: ")
lbl_dist_left_forw.grid(row=2,column=0,sticky = "E")
lbl_dist_left_back = ttk.Label(master=frm_show_data,text="Avstånd bakre vänster: ")
lbl_dist_left_back.grid(row=3,column=0,sticky = "E")
#Hjulsensor
lbl_dist_wheel = ttk.Label(master=frm_show_data,text="Avstånd färdat: ")
lbl_dist_wheel.grid(row=6,column=0,sticky = "E")

##Övriga KÖRDATA
lbl_angle = ttk.Label(master=frm_show_data,text="Vinkel: ")
lbl_angle.grid(row=1,column=2,sticky = "E")

lbl_left_speed = ttk.Label(master=frm_show_data,text="Högra hastigheten: ")
lbl_left_speed.grid(row=4,column=0,sticky = "E")
lbl_right_speed = ttk.Label(master=frm_show_data,text="Högra hastigheten: ")
lbl_right_speed.grid(row=4,column=2,sticky = "E")

lbl_breaker = ttk.Label(master=frm_show_data,text="Autonomt läge eller manuellt:")
lbl_breaker.grid(row=5,column=0,sticky = "E")

lbl_tape = ttk.Label(master=frm_show_data,text="Avvikelse från tejp: ")
lbl_tape.grid(row=5,column=2,sticky = "E")

############### Datan som ska uppdateras
lbl_dist_forw_data = ttk.Label(master=frm_show_data,text="null")
lbl_dist_forw_data.grid(row=1,column=1,sticky = "W")

lbl_dist_right_forw_data = ttk.Label(master=frm_show_data,text="null")
lbl_dist_right_forw_data.grid(row=2,column=3,sticky = "W")
lbl_dist_right_back_data = ttk.Label(master=frm_show_data,text="null")
lbl_dist_right_back_data.grid(row=3,column=3,sticky = "W")
#vänster
lbl_dist_left_forw_data = ttk.Label(master=frm_show_data,text="null")
lbl_dist_left_forw_data.grid(row=2,column=1,sticky = "W")
lbl_dist_left_back_data = ttk.Label(master=frm_show_data,text="null")
lbl_dist_left_back_data.grid(row=3,column=1,sticky = "W")
#Hjulsensor
lbl_dist_wheel_data = ttk.Label(master=frm_show_data,text="null")
lbl_dist_wheel_data.grid(row=6,column=1,sticky = "W")


##Övriga KÖRDATA
lbl_angle_data = ttk.Label(master=frm_show_data,text="null")
lbl_angle_data.grid(row=1,column=3,sticky = "W")

lbl_left_speed_data = ttk.Label(master=frm_show_data,text="null")
lbl_left_speed_data.grid(row=4,column=1,sticky = "W")
lbl_right_speed_data = ttk.Label(master=frm_show_data,text="null")
lbl_right_speed_data.grid(row=4,column=3,sticky = "W")

lbl_breaker_data = ttk.Label(master=frm_show_data,text="null")
lbl_breaker_data.grid(row=5,column=1,sticky = "W")

lbl_tape_data = ttk.Label(master=frm_show_data,text="null")
lbl_tape_data.grid(row=5,column=3,sticky = "W")

##########################################
#   Frame: frm_steeringctl
##########################################
frm_steeringctl = ttk.Frame(window)
frm_steeringctl.grid(column=2, row=1)


chevron_icon = Image.open('gui/resources/chevron-up.png').resize((30, 30))
forward_drive_icon = ImageTk.PhotoImage(chevron_icon)
rotate_left_icon = ImageTk.PhotoImage(chevron_icon.rotate(90, expand=1))
reverse_drive_icon = ImageTk.PhotoImage(chevron_icon.rotate(180))
rotate_right_icon = ImageTk.PhotoImage(chevron_icon.rotate(270, expand=1))
chevron_icon_red = Image.open('gui/resources/chevron-up_red.png').resize((30, 30))
forward_drive_icon_red = ImageTk.PhotoImage(chevron_icon_red)
rotate_left_icon_red = ImageTk.PhotoImage(chevron_icon_red.rotate(90, expand=1))
reverse_drive_icon_red = ImageTk.PhotoImage(chevron_icon_red.rotate(180))
rotate_right_icon_red = ImageTk.PhotoImage(chevron_icon_red.rotate(270, expand=1))

frm_steeringctl.focus_set()
lbl_forward_drive = ttk.Label(frm_steeringctl, image=forward_drive_icon)
lbl_forward_drive.grid(column=1, row=0)
lbl_rotate_left = ttk.Label(frm_steeringctl, image=rotate_left_icon)
lbl_rotate_left.grid(column=0, row=1)
lbl_reverse_drive = ttk.Label(frm_steeringctl, image=reverse_drive_icon)
lbl_reverse_drive.grid(column=1, row=2)
lbl_rotate_right = ttk.Label(frm_steeringctl, image=rotate_right_icon)
lbl_rotate_right.grid(column=2, row=1)
steeringctl = SteeringCtl({
                            'forward_drive':SteeringLabel(lbl_forward_drive, forward_drive_icon_red, forward_drive_icon),
                            'rotate_left':SteeringLabel(lbl_rotate_left, rotate_left_icon_red, rotate_left_icon),
                            'rotate_right':SteeringLabel(lbl_rotate_right, rotate_right_icon_red, rotate_right_icon),
                            'reverse_drive':SteeringLabel(lbl_reverse_drive, reverse_drive_icon_red, reverse_drive_icon)
                            })


######################################################################
#       Frame: explaination of mapcolours
##########################################################################
frm_expl = ttk.Frame(window)
frm_expl.grid(column = 2, row = 0)

frm_canvas_frame = ttk.Frame(frm_expl)
frm_canvas_frame.grid(column = 0, row = 0, rowspan= 7)
cnv_colours = Canvas(frm_canvas_frame,height = 200, width = 30)

colours = ['black','blue','white','red','yellow','purple','green']

cnv_colours.create_rectangle(5,5,20,20,fill = colours[0])
cnv_colours.create_rectangle(5,30,20,45,fill = colours[1])
cnv_colours.create_rectangle(5,60,20,75,fill = colours[2])
cnv_colours.create_rectangle(5,90,20,105,fill = colours[3])
cnv_colours.create_rectangle(5,120,20,135,fill = colours[4])
cnv_colours.create_rectangle(5,150,20,165,fill = colours[5])
cnv_colours.create_rectangle(5,180,20,195,fill = colours[6])


cnv_colours.grid(column = 0,row = 0)

lbl_black = ttk.Label(master=frm_expl,text="Outforskat/vägg")
lbl_black.grid(column=1, row=0,sticky = 'W')
lbl_blue = ttk.Label(master=frm_expl,text="korridor har ej passerat.")
lbl_blue.grid(column=1, row=1,sticky = 'W')
lbl_white = ttk.Label(master=frm_expl,text="korridor som har passerat.")
lbl_white.grid(column=1, row=2,sticky = 'W')
lbl_red = ttk.Label(master=frm_expl,text="Robotens position.")
lbl_red.grid(column=1, row=3,sticky = 'W')
lbl_yellow = ttk.Label(master=frm_expl,text="Nödställds position.")
lbl_yellow.grid(column=1, row=4,sticky = 'W')
lbl_purple = ttk.Label(master=frm_expl,text="Planerad väg.")
lbl_purple.grid(column=1, row=5,sticky = 'W')
lbl_green = ttk.Label(master=frm_expl,text="Nuvarand kortaste väg.")
lbl_green.grid(column=1, row=6,sticky = 'W')






#######################################################################
#       BINDINGS
#######################################################################
window.bind('<Destroy>', on_closing)
window.bind('<KeyPress>', steeringctl.key_pressed)
window.bind('<KeyRelease>', steeringctl.key_released)

os.system('xset r off') #This setting is Linux specific unfortunately.

###############################################################################
def get_map():
    global sensor_data_path
    ROW_LEN = 102
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

    interpreted_map=[]
    print(map[:100])
    for y in range(51):
        interpreted_map.append([])
        for x in range(101):
            mapdata = int.from_bytes(map[x+y*101:x+y*101 +1], byteorder='big', signed=False)
            mask_data = mapdata & 3
            interpreted_map[y].append(mask_data)
    print(interpreted_map[0])
    return interpreted_map


#Denna ska rita ut postionen på kartan
def draw_pos(lables):
    global sensor_data_path
    time = datetime.now().strftime("kl%H_dag%d_%h")
    distressed_last_row_row = ""
    distressed_last_row_column = ""
    robot_last_row_row=""
    robot_last_row_column=""

    for i in range(4):
        try:
            file_path = sensor_data_path + file_name[22 + i].format(time)
            with open(file_path) as csvfile:
                position = csv.reader(csvfile,delimiter = ' ', quotechar='|')
                for row in position:
                    if i == 0:
                        distressed_last_row_row = row
                    elif i == 1:
                        distressed_last_row_column = row
                    if i == 2:
                        robot_last_row_row = row
                    elif i == 3:
                        robot_last_row_column = row
        except:
            print("något gick fel inne i draw_pos")
            continue

    if len(distressed_last_row_row) > 0:
        d_row_list = distressed_last_row_row[0].split(',')
        d_row_pos = int(d_row_list[1])
        d_column_list =distressed_last_row_column[0].split(',')
        d_column_pos = int(d_column_list[1])
        if d_row_pos <= 50 and d_column_pos <= 100:
            canvas.itemconfig(lables[d_row_pos][d_column_pos],fill='yellow') #####här finns positionen

    if len(robot_last_row_row) > 0:
        row_list = robot_last_row_row[0].split(',')
        row_pos = int(row_list[1])
        column_list = robot_last_row_column[0].split(',')
        column_pos = int(column_list[1])
        if row_pos <= 50 and column_pos <= 100:
            canvas.itemconfig(lables[row_pos][column_pos],fill='red') ###########här finns postionen
        return [row_pos,column_pos]


def get_facing():
    global sensor_data_path
    time = datetime.now().strftime("kl%H_dag%d_%h")
    facing_file = sensor_data_path + file_name[26].format(time)
    try:
        with open(facing_file) as csvfile:
            robot_facing = csv.reader(csvfile,delimiter = ' ', quotechar='|')
            for item in robot_facing:
                latest_direction = item
            print("detta är ritkningen {}".format(latest_direction))
            direction = latest_direction[0].split(',')
            return (int(direction[1]))
    except:
        return 0



def draw_nearest_path(lables):
    global sensor_data_path
    time = datetime.now().strftime("kl%H_dag%d_%h")
    robot_facing = []
    latest_direction = ""

    for i in range(2):
        current_facing = 0
        if i == 0:
            current_facing = get_facing()

        file_path = sensor_data_path + file_name[19 + i].format(time)
        facings = [0,1,2,3]
        colours = ['purple','green']
        latest_instruction = ""
        current_pos = draw_pos(lables)
        start_pos =[current_pos,[1,51]]

        try:
            with open(file_path) as csvfile:
                file = csv.reader(csvfile,delimiter = ' ', quotechar='|')

                for row in file:
                    latest_instructions = row
                instruction_vector = latest_instructions[0].split(',')

                if len(instruction_vector) > 1:
                    instruction_vector = instruction_vector[1:]
                    last_pos = [[],[]]
                    last_pos = start_pos
                    if i == 1:
                        canvas.itemconfig(lables[last_pos[i][0]][last_pos[i][1]],fill = colours[i])
                        canvas.itemconfig(lables[last_pos[i][0]-1][last_pos[i][1]],fill = colours[i])
                    for instruction in instruction_vector:
                        print(last_pos[i])
                        if int(instruction) == 0:
                            if current_facing == 0:
                                canvas.itemconfig(lables[last_pos[i][0]+1][last_pos[i][1]],fill = colours[i])
                                canvas.itemconfig(lables[last_pos[i][0]+2][last_pos[i][1]],fill=colours[i])
                                last_pos[i][0] = last_pos[i][0] + 2
                            if current_facing == 1:
                                canvas.itemconfig(lables[last_pos[i][0]][last_pos[i][1]-1],fill= colours[i])
                                canvas.itemconfig(lables[last_pos[i][0]][last_pos[i][1]-2],fill = colours[i])
                                last_pos[i][1] = last_pos[i][1] - 2
                            if current_facing == 2:
                                print(last_pos[i][0])
                                canvas.itemconfig(lables[last_pos[i][0] - 1][last_pos[i][1]],fill = colours[i])
                                canvas.itemconfig(lables[last_pos[i][0]-2][last_pos[i][1]], fill = colours[i])
                                last_pos[i][0] = last_pos[i][0] - 2
                            if current_facing == 3:
                                canvas.itemconfig(lables[last_pos[i][0]][last_pos[i][1]+1],fill=colours[i])
                                canvas.itemconfig(lables[last_pos[i][0]][last_pos[i][1]+2],fill=colours[i])
                                last_pos[i][1] = last_pos[i][1] + 2

                        if int(instruction) ==2:
                            if current_facing == 0:
                                current_facing = 3
                            else:
                                current_facing -= 1
                        if int(instruction) == 1:
                            if current_facing == 3:
                                current_facing = 0
                            else:
                                current_facing += 1

                        if int(instruction) == 3:
                            if current_facing == 2:
                                current_facing = 0
                            elif current_facing == 3:
                                current_facing = 1
                            else:
                                current_facing += 2
        except:
            return




def draw_map(lables):
    map = get_map()
    # image1 = Image.open("wall_left.png")
    # test = ImageTk.PhotoImage(image1)
    colour = 'black'

    for i in range(51):
        for j in range(101):
            # print("rad {}".format(i))
            # print("kolumn {}".format(j))
            if map[i][j] == 3:
                colour = 'white'
            # elif map[i][j] == 2:
            #     colour = 'pink'
            elif map[i][j] == 1:
                colour = 'blue'
            elif map[i][j] == 0:
                colour = 'black'
            canvas.itemconfig(lables[i][j],fill=colour)


def update_colour():
    for i in range(51):
        for j in range(51):
            lables[i][j]['bg'] = 'white'


frm_map = Frame(window, bg='black')
canvas = Canvas(frm_map,height=5*26+15*25,width=5*51+15*50)
i = 0
list = []
coordy = 5*26+15*25
coordx = 0
for r in range(51):
    list.append([])
    right_cornery = coordy
    coordy = coordy - 15*(r % 2) - 5*((r+1) % 2)
    coordx = 0
    for c in range(101):
        right_cornerx = coordx
        coordx = coordx + 15*(c % 2) + 5*((c + 1) % 2)
        coords = (coordx, coordy,right_cornerx , right_cornery)

        rectangle = canvas.create_rectangle(coords, fill='black')
        list[r] = [rectangle] + list[r]
    i = i + 1
frm_map.grid(row=0,column=0, columnspan=2,rowspan = 2  )

#Funtion that meakes sure that all map information is updated at the same time
def update_map(list):
    update_drive_data()
    print("inne i kartuppdatering")
    draw_map(list)
    draw_nearest_path(list)
    draw_pos(list)

    window.after(500, update_map,list)

canvas.grid(row=0,column=0)
###############################################################################



#window.after(1000,update_drive_data)
window.after(1000,update_map,list)


for frame in window.winfo_children():
    frame['borderwidth'] = 2
    frame['relief'] = 'groove'
window.mainloop()
