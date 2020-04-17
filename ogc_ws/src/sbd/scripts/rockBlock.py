#!/usr/bin/env python3

#    Copyright 2015 Makersnake
# 
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
# 
#    Original code: https://github.com/MakerSnake/pyRockBlock
   
import glob
import signal
import sys
import time

import serial

class rockBlockProtocol(object):
    
    def rockBlockConnected(self):pass
    def rockBlockDisconnected(self):pass
    
    #SIGNAL
    def rockBlockSignalUpdate(self,signal):pass
    def rockBlockSignalPass(self):pass
    def rockBlockSignalFail(self):pass
    
    #MT
    def rockBlockRxStarted(self):pass
    def rockBlockRxFailed(self):pass
    def rockBlockRxReceived(self,mtmsn,data):pass
    def rockBlockRxMessageQueue(self,count):pass
     
    #MO
    def rockBlockTxStarted(self):pass
    def rockBlockTxFailed(self):pass
    def rockBlockTxSuccess(self,momsn):pass
    
class rockBlockException(Exception):
    pass
    
class rockBlock(object):
    
    IRIDIUM_EPOCH = 1399818235000 # May 11, 2014, at 14:23:55 (This will be 're-epoched' every couple of years!)
        
    def __init__(self, portId, callback):
        '''
        Initialize serial connection to Rockblock. If unable to connect to Rockblock, exception will be raised
        '''
        self.s = None
        self.portId = portId
        self.callback = callback
        self.autoSession = True # When True, we'll automatically initiate additional sessions if more messages to download
        
        try:
            self.s = serial.Serial(self.portId, 19200, timeout=5)
            if( self._configurePort() ):
                self.ping() # KEEP SACRIFICIAL!
                self.s.timeout = 60
                if( self.ping() ):
                    if(self.callback != None and callable(self.callback.rockBlockConnected) ):   
                        self.callback.rockBlockConnected()
                        return
            self.close()
            raise rockBlockException()
        except (Exception):
            raise rockBlockException
        
    
    def ping(self):
        '''Ensure that the connection is still alive'''
        self._ensureConnectionStatus()
        command = "AT"
        self.s.write((command + "\r").encode())
        if( self.s.readline().strip().decode() == command ):
            if( self.s.readline().strip().decode() == "OK" ):                   
                return True
        return False


    def pingception(self):
        '''Handy function to check the connection is still alive, else throw an Exception'''
        self._ensureConnectionStatus()
        self.s.timeout = 5
        if(self.ping() == False):
            raise rockBlockException
        self.s.timeout = 60
            
    
    def requestSignalStrength(self):
        '''Return the Iridium Signal Strength; if unsuccesful, return -1'''
        self._ensureConnectionStatus()
        command = "AT+CSQ"
        self.s.write((command + "\r").encode())
        if( self.s.readline().strip().decode() == command):
            response = self.s.readline().strip().decode()
            if( response.find("+CSQ") >= 0 ):
                self.s.readline().strip()   #OK
                self.s.readline().strip()   #BLANK
                if( len(response) == 6):
                    return int( response[5] )
        return -1   
     
    
    def messageCheck(self):
        '''Check for incoming MT msgs, and return true if successful'''
        self._ensureConnectionStatus()
        if(self.callback != None and callable(self.callback.rockBlockRxStarted) ):
            self.callback.rockBlockRxStarted()
        if( self._attemptConnection() and self._attemptSession() ):
            return True
        else:
            if(self.callback != None and callable(self.callback.rockBlockRxFailed) ):
                self.callback.rockBlockRxFailed()
                
        
    def networkTime(self):
        '''Return the Iridium network time; if unable to get valid time, return 0'''
        self._ensureConnectionStatus()
        command = "AT-MSSTM"
        self.s.write((command + "\r").encode())
        if(self.s.readline().strip().decode() == command):
            response = self.s.readline().strip().decode()
            self.s.readline().strip()  #BLANK
            self.s.readline().strip()  #OK
            if( not "no network service" in response ):
                utc = int(response[8:], 16)
                utc = int((self.IRIDIUM_EPOCH + (utc * 90))/1000)
                return utc
            else:
                return 0
                      
                            
    def sendMessage(self, msg):
        '''Send an MT msg, and return True/False depending on whether attempt was successful'''
        self._ensureConnectionStatus()

        if(self.callback != None and callable(self.callback.rockBlockTxStarted) ):
            self.callback.rockBlockTxStarted()

        if( self._queueMessage(msg) and self._attemptConnection()  ):
            # Try a max of 3 unsuccessful times before giving up permanently
            SESSION_DELAY = 1
            SESSION_ATTEMPTS = 3
            while(True):
                SESSION_ATTEMPTS = SESSION_ATTEMPTS - 1
                if(SESSION_ATTEMPTS == 0):
                    break
                if( self._attemptSession() ):
                    print ("Yay!") # Replace with a better msg...
                    return True
                else:
                    time.sleep(SESSION_DELAY)       
        if(self.callback != None and callable(self.callback.rockBlockTxFailed) ):
            self.callback.rockBlockTxFailed()
        return False
    
    
    def getSerialIdentifier(self):
        '''Query Rockblock's IMEI number'''
        self._ensureConnectionStatus()
        command = "AT+GSN"
        self.s.write((command + "\r").encode())
        if(self.s.readline().strip().decode() == command):
            response = self.s.readline().strip().decode()
            self.s.readline().strip()  #BLANK
            self.s.readline().strip() #OK
            return response
    

    def setup(self):
        '''
        One-time initial setup function (Disables Flow Control)
        This only needs to be called once, as is stored in non-volitile memory
        Make sure you DISCONNECT RockBLOCK from power for a few minutes after this command has been issued...
        '''
        self._ensureConnectionStatus()
        # Disable Flow Control
        command = "AT&K0"
        self.s.write((command + "\r").encode())
        if(self.s.readline().strip().decode() == command and self.s.readline().strip().decode() == "OK"):
            # Store Configuration into Profile0
            command = "AT&W0"
            self.s.write((command + "\r").encode())
            if(self.s.readline().strip().decode() == command and self.s.readline().strip().decode() == "OK"):
                # Use Profile0 as default
                command = "AT&Y0"
                self.s.write((command + "\r").encode())
                if(self.s.readline().strip().decode() == command and self.s.readline().strip().decode() == "OK"):
                    # Flush Memory
                    command = "AT*F"
                    self.s.write((command + "\r").encode())
                    if(self.s.readline().strip().decode() == command and self.s.readline().strip().decode() == "OK"):
                        # self.close()
                        return True
                    
        
        
        return False        
    
    def close(self):
        '''Close Rockblock serial connection'''
        if(self.s != None):
            self.s.close()
            self.s = None
    
     
    @staticmethod
    def listPorts():
        '''Handy function to list all available ports (to find out where Rockblock is connected'''
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        result = []
        
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        
        return result
    
        
    #Private Methods - Don't call these directly!
    def _queueMessage(self, msg):
        '''Prepare a Mobile-Originated (MO) msg'''
        self._ensureConnectionStatus()
                
        if( len(msg) > 50):
            print ("sendMessageWithBytes bytes should be <= 50 bytes")
            return False
        
        command = "AT+SBDWB=" + str( len(msg) )
        self.s.write((command + "\r").encode())
        
        if(self.s.readline().strip().decode() == command):
            if(self.s.readline().strip().decode() == "READY"):
                checksum = 0
                for c in msg:
                    checksum = checksum + ord(c)   
                self.s.write( str(msg).encode() )
                self.s.write( chr( checksum >> 8 ).encode() )
                self.s.write( chr( checksum & 0xFF ).encode() )
                self.s.readline().strip()  #BLANK
                result = False 
                if(self.s.readline().strip().decode() == "0"):
                    result = True                 
                self.s.readline().strip()  #BLANK
                self.s.readline().strip() #OK
                return result
        return False
    
    
    def _configurePort(self):
        '''Initial setup of the serial port when opening connection to Rockblock'''
        if( self._enableEcho() and self._disableFlowControl and self._disableRingAlerts() and self.ping() ):         
            print("config port successful")
            return True
        else:
            print("config port unsuccessful")
            return False
        
        
    def _enableEcho(self):
        '''Turn on Echo using ATE1 command'''
        self._ensureConnectionStatus()
        command = "ATE1"
        self.s.write((command + "\r").encode())
        response = self.s.readline().strip().decode()
        if(response == command or response == ""):     
            if( self.s.readline().strip().decode() == "OK" ):
                return True
        return False
    
    
    def _disableFlowControl(self):
        '''Disable Flow Control using AT&K0 command'''
        self._ensureConnectionStatus()
        command = "AT&K0"
        self.s.write((command + "\r").encode())
        if(self.s.readline().strip().decode() == command):
            if( self.s.readline().strip().decode() == "OK" ):
                return True
        return False
    
    
    def _disableRingAlerts(self):
        '''Disable ring alerts using AT+SBDMTA=0'''
        self._ensureConnectionStatus()
        command = "AT+SBDMTA=0"
        self.s.write((command + "\r").encode())
        if( self.s.readline().strip().decode() == command ):
            if( self.s.readline().strip().decode() == "OK" ):
                return True
        return False
                 
                 
    def _attemptSession(self):
        '''Try to establish an Iridium SBD session and perform mailbox check. Works for both MO and MT msgs'''

        self._ensureConnectionStatus()
        SESSION_ATTEMPTS = 3
                
        while(True):
            
            if(SESSION_ATTEMPTS == 0):
                return False            
            
            SESSION_ATTEMPTS = SESSION_ATTEMPTS - 1
            command = "AT+SBDIX"
            self.s.write((command + "\r").encode())
            
            if( self.s.readline().strip().decode() == command ):
                response = self.s.readline().strip().decode()
                if( response.find("+SBDIX:") >= 0 ):
                    self.s.readline()   #BLANK
                    self.s.readline()   #OK    

                    # Mailbox check response: +SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>
                    response = response.replace("+SBDIX: ", "")                
                    parts = response.split(",")
                    moStatus = int(parts[0])
                    moMsn = int(parts[1])
                    mtStatus = int(parts[2])
                    mtMsn = int(parts[3])
                    mtLength = int(parts[4])
                    mtQueued = int(parts[5])
                    
                    # Check for Mobile-Originated msgs
                    if(moStatus <= 4):
                        self._clearMoBuffer()
                        if(self.callback != None and callable(self.callback.rockBlockTxSuccess) ):   
                            self.callback.rockBlockTxSuccess( moMsn )
                        pass
                    else:
                        if(self.callback != None and callable(self.callback.rockBlockTxFailed) ): 
                            self.callback.rockBlockTxFailed()
                    
                    # SBD message successfully received from the GSS.
                    if(mtStatus == 1 and mtLength > 0): 
                        self._processMtMessage(mtMsn)
                    
                    # AUTOGET NEXT MESSAGE
                    if(self.callback != None and callable(self.callback.rockBlockRxMessageQueue) ): 
                        self.callback.rockBlockRxMessageQueue(mtQueued)
                    
                    #There are additional MT messages to queued to download
                    if(mtQueued > 0 and self.autoSession == True):
                        self._attemptSession()
                    
                    if(moStatus <= 4):                     
                        return True

        return False
     
        
    def _attemptConnection(self):
        '''Ensure valid network time and sufficient signal strength'''
        self._ensureConnectionStatus()

        TIME_ATTEMPTS = 20
        TIME_DELAY = 1
       
        SIGNAL_ATTEMPTS = 10
        RESCAN_DELAY = 10                 
        SIGNAL_THRESHOLD = 2
        
        # Wait for valid Network Time
        while True:
            if(TIME_ATTEMPTS == 0):
                if(self.callback != None and callable(self.callback.rockBlockSignalFail) ): 
                    self.callback.rockBlockSignalFail()
                return False
            if( self._isNetworkTimeValid() ):
                break
            TIME_ATTEMPTS = TIME_ATTEMPTS - 1
            time.sleep(TIME_DELAY)
            
        #Wait for acceptable signal strength (strength of 2 and above considered acceptable)
        while True:
            signal = self.requestSignalStrength()
            if(SIGNAL_ATTEMPTS == 0 or signal < 0):
                print  ("NO SIGNAL")     
                if(self.callback != None and callable(self.callback.rockBlockSignalFail) ): 
                    self.callback.rockBlockSignalFail()
                return False
            self.callback.rockBlockSignalUpdate( signal )
            if( signal >= SIGNAL_THRESHOLD ):
                if(self.callback != None and callable(self.callback.rockBlockSignalPass) ): 
                    self.callback.rockBlockSignalPass()
                return True
            SIGNAL_ATTEMPTS = SIGNAL_ATTEMPTS - 1
            time.sleep(RESCAN_DELAY)
        

    def _processMtMessage(self, mtMsn):
        '''Process incoming Mobile-Terminated (MT) msg'''

        self._ensureConnectionStatus()
        command = "AT+SBDRB"
        self.s.write((command + "\r").encode())
        if (self.s.readline().strip().decode() == command): # Echo
            response = self.s.readline().strip().decode()

            if( response == "OK" ):
                # Blank msg
                print ("No message content.. strange!")
                if(self.callback != None and callable(self.callback.rockBlockRxReceived) ): 
                    self.callback.rockBlockRxReceived(mtMsn, "")
            else:
                # Pass the MT msg to the callback object                                   
                content = response[2:-2]
                if(self.callback != None and callable(self.callback.rockBlockRxReceived) ): 
                    self.callback.rockBlockRxReceived(mtMsn, content)
                self.s.readline()   #BLANK?
                
    
    def _isNetworkTimeValid(self):
        '''Ensure valid network time, otherwise return False'''

        self._ensureConnectionStatus()
        command = "AT-MSSTM"
        self.s.write((command + "\r").encode())

        if( self.s.readline().strip().decode() == command ):  #Echo
            response = self.s.readline().strip().decode()
            if( response.startswith("-MSSTM") ):    #-MSSTM: a5cb42ad / no network service
                self.s.readline()   #OK
                self.s.readline()   #BLANK
                if( len(response) == 16):    
                    return True
        return False
    
    
    def _clearMoBuffer(self):
        '''Clear the Mobile-Originated (MO) Buffer'''
        self._ensureConnectionStatus()
        command = "AT+SBDD0"
        self.s.write((command + "\r").encode())
        if(self.s.readline().strip().decode() == command):
            if(self.s.readline().strip().decode() == "0"):
                self.s.readline()  #BLANK
                if(self.s.readline().strip().decode() == "OK"):
                    return True
        return False
        
        
    def _ensureConnectionStatus(self):
        '''Make sure that the serial connection to Rockblock is open'''
        if(self.s == None or self.s.isOpen() == False):
            raise rockBlockException()