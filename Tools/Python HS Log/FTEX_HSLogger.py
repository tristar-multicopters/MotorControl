#------------------------------------------------------------------------------------------------#
# FTEX Log high speed tool for the PC                                                            #
#                                                                                                #
# Brief: Can manipulate the behavior of the high speed logging module in a controller            #
#        with a PC using a USB<->UART adapter. Main function is to retrieve the logged data      #
#        and insert it in a .csv file. Each time we get new data a new file is created           #
#        The name of the file is simply the number of files in the folder C:\FTEX_HSLogs + 1     #
#------------------------------------------------------------------------------------------------#

import serial.tools.list_ports
import csv
import os
import numpy
from os import listdir
import time
from enum import Enum

class State(Enum):
    
    WAIT_CMD           = 0
    WAIT_DATA          = 1
    PROCESS_DATA_INFO  = 2
    REQUEST_LOG_DATA   = 3
    WRITE_DATA_IN_CSV  = 4 


POSITIVEINT16SIZE = 32767
POSITIVEINT32SIZE = 65535 

global state 
global DataReq        #number of bytes we want to receive
global StatePostReq   #State that we need to go to after receiving the data
global packetList     #used for data reception
packetList   = []  
global nbDataPoints
nbDataPoints = 0 
global nbDatainSet
nbDatainSet = 0 

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portList = []

#header for the .csv file
csvHeader = ['Current A','Current B','Id actual','Iq actual','Id target','Iq target','Electrical Pos','Faults']
filePath = "C:\FTEX_HSLogs"

#///////////////////////////////MAIN_CODE//////////////////////////////////////
def main():
    global state
    state = State.WAIT_CMD
    for onePort in ports:            #list all of the found COM ports 
        portList.append(str(onePort))
        print(str(onePort))

    if len(portList) == 0:   #if there are no ports send an error message and close the tool
        print("No COM port found, press enter to close the tool")    #TODO:Good upgrade would be to enable to user to refresh the list
        input()
        quit()         

    val = input("select Port: COM") #Ask them to choose a port

    for x in range(0, len(portList)):
        if portList[x].startswith("COM" + str(val)):
            portVar = "COM" + str(val)
            print(portList[x])

    serialInst.baudrate = 115200 #Set the baudrate of the usrt port and open it
    serialInst.port = portVar
    serialInst.open()
    i = 0

    while True:
    
        match state: 
            case State.WAIT_CMD: #Get a command from the user
                GetCmd()

            case State.WAIT_DATA: #wait for a number of data to be received and then switch to the next specified state
                WaitForData()

            case State.PROCESS_DATA_INFO: #unpack the data info that we have received
                ProcessDataInfo()

            case State.REQUEST_LOG_DATA:  #setup for the next data reception and tell the controller we are ready
                RequestLogData()

            case State.WRITE_DATA_IN_CSV: #placing the data in the .csv
                WriteInCSV()

            case _:  
                assert False,"Reached a part of code that should be unreachable"

    

###############################################################################
# Name : Get Command
# Brief: Waits for the next command of the user, 
#        when received executes said command
###############################################################################
def GetCmd():
    global state
    global DataReq
    global StatePostReq 
    val = input("Input command:")    

    match str(val):
        case 'g': #get data logged
            print("Requesting Log")
            serialInst.write('dr'.encode('utf-8'))#send the data request
            StatePostReq = State.PROCESS_DATA_INFO
            state = State.WAIT_DATA
            DataReq      = 6
            
        case 'r': #reset log ram
            serialInst.write('fr'.encode('utf-8')) #send the request to Format the ram 
            print("Sent Format ram request")             
            state        = State.WAIT_CMD

        case 's': #Start log
            serialInst.write('sl'.encode('utf-8'))#send the start command
            print("Sent Start log request")             
            state        = State.WAIT_CMD

        case 'e': #end log
            serialInst.write('el'.encode('utf-8'))#send the data request
            print("Sent End log request")
            state        = State.WAIT_CMD

        case 'q': #exit tool
            print("exiting tool")
            quit()      

        case _:
            print("Unkown command")
            print(str(val))
            print("please choose one of the following")
            print("g to get the data logged")
            print("r to reset the logged data") 
            print("s to start the log")
            print("e to end the log")
            print("q to exit que tool") 



###############################################################################
# Name : Wait for data
# Brief: Waits for the reception of UART data. Number of byte expected to get 
#        is in DataReq. Then once the correct amount has been received the 
#        function selects the state present in StatePosReq as the next state. 
###############################################################################
def WaitForData():
    global state
    global DataReq
    global StatePostReq 
    global packetList

    packetList.clear()
    DataReceived = False
    print("Waiting for this amount of data",DataReq)
    while DataReceived == False:
        if serialInst.in_waiting: #do we have data waiting to be read ? 
            dataAsInt = int.from_bytes(serialInst.read(), 'little')
            packetList.append(dataAsInt)
                     
        if len(packetList) == DataReq: #Have we received all of the data that we expected
            state = StatePostReq 
            DataReceived = True

###############################################################################
# Name : Process data info
# Brief: Now that we have received the data info recombine the bytes to learn 
#        how many data per data set and how many data points we have.
#        (each data point contaisn one data set)
###############################################################################
def ProcessDataInfo():
    global state
    global packetList
    global nbDataPoints
    global nbDatainSet

    if packetList[0] == ord('d') and  packetList[1] == ord('i'):
        print("Received data info")
        nbDataPoints = packetList[2] + (packetList[3] << 8) #we have received the data info
        nbDatainSet  = packetList[4] + (packetList[5] << 8) 
               
        #For debugging
        #print("We have this amount of data points")
        #print(nbDataPoints)
        #print("and this many data per set")
        #print(nbDatainSet)
        state = State.REQUEST_LOG_DATA
    else:
        print("Expected data info, dont understand message")
        print(packetList)
        state = State.WAIT_CMD

###############################################################################
# Name : Request log data
# Brief: Asks the controller to send us the data that it has in it's buffer, 
#       
###############################################################################
def RequestLogData():
    global state
    global DataReq
    global StatePostReq 
    global nbDataPoints
    global nbDatainSet

    StatePostReq = State.WRITE_DATA_IN_CSV 
    state = State.WAIT_DATA
    DataReq = (nbDataPoints * nbDatainSet * 2)

    #For debugging
    #print("Expecting this number of bytes",DataReq)

    serialInst.write('rr'.encode('utf-8')) #tell the gan runner that we are ready to receive the log       

###############################################################################
# Name : Write in CSV
# Brief: Writes the data from the buffer inside a new .CSV file
#
###############################################################################
def WriteInCSV():
    global state
    global packetList
    global nbDataPoints
    global nbDatainSet

    print("we have received this number of bytes",len(packetList))

    #finding what is the name of the next .csv file
    all_files = listdir("C:\FTEX_HSLogs")    
    csv_files = list(filter(lambda f: f.endswith('.csv'), all_files))

    if len(csv_files) == 0:  #if no file are present
        newFileName = str(1) + ".csv" 
    else:   
        var = str(len(csv_files)+1)
        newFileName = var + ".csv"
            
    path = os.path.join(filePath, newFileName) #combine the name of the new file with the file path   

    with open(path, 'w', newline='') as csvFile : #writing the data in the .csv  
        csvWriter = csv.writer(csvFile)
        csvWriter.writerow(csvHeader)
                
        row = []    
        for x in range(0,nbDataPoints):    #Take the data from the giant array and write it in the .csv
            for y in range(0,nbDatainSet):

                lowVal  = packetList[((x*2) * nbDatainSet + (y * 2))]
                highVal = (packetList[((x*2) * nbDatainSet + ((y * 2) + 1))] << 8)
                        
                if lowVal + highVal == 0:
                    nextVal = 0
                else:    
                    nextVal = lowVal + highVal
                        
                if nextVal > POSITIVEINT16SIZE:    #manually apply the negative values because python int is 32 bits and GNR int is 16 bits
                            nextVal -= POSITIVEINT32SIZE

                row.append(str(nextVal))

            csvWriter.writerow(row)
            row.clear()
    print("Data succesfully transfered into the .csv")
    state        = State.WAIT_CMD 
    
main()
    
          
