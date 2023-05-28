## Import Directories

import serial
import PySimpleGUI as sg
import math
import time

## Initialize Serial Connection
ser = serial.Serial("COM3", 9600)

## Set GUI Theme
sg.theme("Black")

## Initialize Variables
IDs = []
col = []
cc = str()

run = True
t_buf = 0
timer = 0
t = time.time()

## Collect Unique IDs
while run:
    
    # get time
    t_buf = t
    t = time.time()
    
    # read serial from arduino
    cc = str(ser.readline())
        
    if len(cc) != 36:
        continue
    
    cc = cc[2:][:-5]
    
    cc_sp = cc.split(' ')
    
    ID = hex(int("0x" + cc_sp[0],16))
    message = cc_sp[2:]
    
    # add to list if new
    if ID not in IDs:
        IDs.append(ID)
        #print(ID)
        timer = 0
    else:
        timer = timer + t-t_buf
        
    # exit loop if no new IDs found for a full second
    if timer > 2:
        run = False
        
# sort ids
IDs.sort()

# split IDs into columns
num_id = len(IDs)
num_row = math.ceil(num_id/2)

IDs1 = IDs[:num_row]
IDs2 = IDs[num_row:]

## Create Interface

# title line
title = [sg.Text("CAN Bus Monitor",size=(1000,1),justification='center',font=('Arial Bold',22),key='Title')]

# header line
header = [sg.Text("ID",size=(10,1),justification='left')]
for l in 'ABCDEFGH':
    header.append(sg.Text(l,size=(5,1),justification='center'))
header.append(sg.VSeparator())
header.append(sg.Text("ID",size=(10,1),justification='left'))
for l in 'ABCDEFGH':
    header.append(sg.Text(l,size=(5,1),justification='center'))
    
# build rows
for i, id in enumerate(IDs1):
    lay_buf = [sg.Text(hex(int(IDs1[i],16)),size=(10,1),justification='left',key=IDs1[i])]

    for j in range(8):
        lay_buf.append(sg.Text(j,size=(5,1),justification='center',key=IDs1[i]+str(j)))
        
    if i < len(IDs2):
        lay_buf.append(sg.VSeparator())
        lay_buf.append(sg.Text(hex(int(IDs2[i],16)),size=(10,1),justification='left',key=IDs2[i]))

        for j in range(8):
            lay_buf.append(sg.Text(j,size=(5,1),justification='center',key=IDs2[i]+str(j)))
        
    col = [col,
            [lay_buf]]

# build the layout
layout = [[title],
            [header],
            [sg.HorizontalSeparator()],
            [col],
            [sg.Push(),sg.Button('Start',key='Start_Button'),sg.Button('Stop',key='Stop_Button'),sg.Exit(),sg.Push()]]

# create the interface
window = sg.Window('CAN Monitor', layout, size=(1400,700), no_titlebar=False, auto_size_buttons=False, keep_on_top=True, font=("Arial",10),grab_anywhere=True)

# wait for user input
run = False
while True:
    
    # refresh window
    event, values, = window.read(timeout=10)
    
    # wait for button press
    if event in (sg.WIN_CLOSED, 'Exit'):    # close window
        break
    if event == 'Start_Button': # start reading can bus
        run = True
    if event == 'Stop_Button':  # stop reading can bus
        run = False
    
    # read can bus
    if run:
        
        # read serial from arduino
        cc = str(ser.readline())
            
        if len(cc) != 36:
            continue
        
        cc = cc[2:][:-5]
        
        cc_sp = cc.split(' ')
        
        ID = hex(int("0x" + cc_sp[0],16))
        message = cc_sp[2:]
        
        # find ID and update messages
        if ID in IDs:
            row = IDs.index(ID)
            for i, byte in enumerate(message):
                window[ID + str(i)].update(byte)