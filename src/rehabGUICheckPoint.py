# Before Running Main, run this line in terminal
# pip install -r requirements.txt
# Library/Dependencies setup
import PySimpleGUI as sg      
import serial as sr
from time import sleep
from threading import Thread
# TODO
# repetition input

# main window setup function
def mainWindowSetup():
    sg.theme('DarkAmber')    # Keep things interesting for your users

    layout = [[sg.Text('Conenction Setup'),sg.Text('Robot Joints'),sg.Text('Target Position'),sg.Text('Current Position'),sg.Text('Load Trajectory')],      
            [sg.Text('Arduino Serial Port',size=(15,1), key='-APort-'),sg.Text('       1          '),sg.Input(key='-T1-',size=(11, 1)), sg.Text('0',size=(13,1), key='-R1-'),sg.Input(key='-Traj-',size=(12, 1))],
            [sg.Input(key='-SrPort-',size=(14, 1)),sg.Text('         2          '),sg.Input(key='-T2-',size=(11, 1)), sg.Text('0',size=(13,1), key='-R2-'),sg.Button('       Load       ',key='-LoadTraj-')],
            [sg.Button('      Connect       ',key='-Conn-'),sg.Text('         3          '),sg.Input(key='-T3-',size=(11, 1)), sg.Text('0',size=(13,1), key='-R3-'),sg.Button('    Calibrate    ',key='-Calibrate-')],      
            [sg.Exit('          Exit          ',key='-Exit-'),sg.Text('        .            '),sg.Button('      Send      ',key='-Send-')]]      

    window = sg.Window('Ankle Rehabilitation Interface', layout)
    return window
# Serial Comms fucntion
def serialSetup(port):
    try:
        comm = sr.Serial(port,9600,timeout=1)
        return comm
    except:
        print("Serial Port not available")
# Serial Read Thread
def serialComms(window,comm,send_flag,stop):
    while True:
        # print('thread running')
        if(comm and send_flag == False):
            srData = comm.readline()
            if(srData):
                srData = srData.decode()
                srData = srData.strip()
                lineSplit = srData.split(",") #split the line into a list
                #Parse integer values and assign variables from the list: 
                # m1Real=float(lineSplit[0])
                # m2Real=float(lineSplit[1])
                # m3Real=float(lineSplit[2])
                window['-R1-'].update(lineSplit[0])
                window['-R2-'].update(lineSplit[1])
                window['-R3-'].update(lineSplit[2])
                sleep(0.1)
                # print(m1Real,m2Real,m3Real)
        if stop():
            print('thread stop')
            break
# Serial Write Function
def serialWrite(comm,data):
    comm.write(data) 
# Main function
def main():
    # Local Variables
    stop_threads = False
    send_flag = False
    sendBuffer=[0,0,0]
    # Generate main window
    mainWindow=mainWindowSetup()
    while True:                             # The Event Loop
        event, values = mainWindow.read() 
        # print(event, values)    
        if(event == sg.WIN_CLOSED or event == '-Exit-'):
            try:
                stop_threads = True
                t1.join()
                commVar.close()
            except:
                print("No Serial Port to close")
            break   
        # Serial setup
        if event == '-Conn-':
            port = values['-SrPort-']
            if(port == ""):
                port = "/dev/tty.usbmodem142301"
            commVar = serialSetup(port)
            t1 = Thread(target = serialComms, args =(mainWindow,commVar,send_flag,lambda : stop_threads, ))
            t1.start()
        if event == '-Send-':
            try:
                sendBuffer[1] = values['-T1-']
                sendBuffer[2] = values['-T2-']
                sendBuffer[3] = values['-T3-']
                serialWrite(commVar,sendBuffer)
            except:
                print("Cant send data")
    # Program closed    
    mainWindow.close()

if __name__ == "__main__":
    main()