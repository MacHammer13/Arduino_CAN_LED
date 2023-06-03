## Import Directories

import serial
import tkinter as tk
import tkinter.ttk as ttk

def start_monitor():
    
    global run 
    
    port = ent_port.get()
    baud = ent_baud.get()
    
    if len(port) == 0:
        print("Please define port")
        run = False
        return
    if len(baud) == 0:
        print("Please define baud rate")
        run = False
        return
    else:
        baud = int(baud)
        run = True
    
    global ser 
    ser = serial.Serial(port,baud)
    
def stop_monitor():
    global run
    run = False
    ser.close()
    print(run)

def exit_monitor():
    main_window.quit()
    
def serial_read():
    
    if run:
        cc = str(ser.readline())
        
        if len(cc) != 36:
            main_window.after(1,serial_read)
            return
        
        cc = cc[2:][:-5]
        
        cc_sp = cc.split(' ')
        
        ID = "0x" + cc_sp[0]
        message = cc_sp[2:]
        
        if ID not in IDs:
            IDs.append(ID)
            frame_id = tk.Frame(master=main_window)
            tk.Label(master=frame_id,text=ID,width=10,height=1,font=("Arial",14)).pack(side=tk.LEFT,padx=5,pady=5)
            messages = list()
            for i, byte in enumerate(message):
                lbl = tk.Label(master=frame_id,text=byte,width=5,height=1,font=("Arial",14))
                lbl.pack(side=tk.LEFT,padx=5,pady=5)
                messages.append(lbl)
            message_labels.append(messages)
            frame_id.pack(side=tk.TOP)
        else:
            for j, id in enumerate(IDs):
                if id == ID:
                    for i, byte in enumerate(message):
                        x = message_labels[j][i].config(text=byte)
                        
    main_window.after(1,serial_read)

IDs = []
ser = []
run = False
message_labels = list()

# create GUI window
main_window = tk.Tk()

title = tk.Label(master=main_window,text="CAN Bus Monitor",font=('Arial Bold',20))

frame_ent = tk.Frame(master=main_window)
frame_btn = tk.Frame(master=main_window)
frame_head = tk.Frame(master=main_window)

lbl_port = tk.Label(master=frame_ent,text="Port Name",width=10,height=1,font=('Arial',14))
lbl_baud = tk.Label(master=frame_ent,text="Baud Rate",width=10,height=1,font=('Arial',14))
ent_port = tk.Entry(master=frame_ent,width=15,font=('Arial',14))
ent_port.insert(0,"COM3")
ent_baud = tk.Entry(master=frame_ent,width=15,font=('Arial',14))
ent_baud.insert(0,"9600")

lbl_port.pack(side=tk.LEFT,padx=5,pady=5)
ent_port.pack(side=tk.LEFT,padx=5,pady=5)
lbl_baud.pack(side=tk.LEFT,padx=5,pady=5)
ent_baud.pack(side=tk.LEFT,padx=5,pady=5)

btn_start = tk.Button(master=frame_btn,text="Start",width=10,height=1,font=('Arial Bold',14),command=start_monitor)
btn_stop = tk.Button(master=frame_btn,text="Stop",width=10,height=1,font=('Arial Bold',14),command=stop_monitor)
btn_exit = tk.Button(master=frame_btn,text="Exit",width=10,height=1,font=('Arial Bold',14),command=exit_monitor)

btn_start.pack(side=tk.LEFT,padx=5,pady=5)
btn_stop.pack(side=tk.LEFT,padx=5,pady=5)
btn_exit.pack(side=tk.LEFT,padx=5,pady=5)

lbl_ID_head = tk.Label(master=frame_head,text="ID",width=10,height=1,font=('Arial',14))
lbl_ID_head.pack(side=tk.LEFT,padx=5,pady=5)
for l in "ABCDEFGH":
    tk.Label(master=frame_head,text=l,width=5,height=1,font=('Arial',14)).pack(side=tk.LEFT,padx=5,pady=5)

title.pack(side=tk.TOP)
frame_ent.pack(side=tk.TOP)
frame_btn.pack(side=tk.TOP)
frame_head.pack(side=tk.TOP)
ttk.Separator(master=main_window,orient="horizontal").pack(side=tk.TOP,fill="x")

main_window.after(1,serial_read())
main_window.mainloop()