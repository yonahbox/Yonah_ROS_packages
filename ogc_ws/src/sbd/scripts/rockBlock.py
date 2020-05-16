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

'''
rockBlock: Module to interface directly with Rockblock satellite modem through a serial connection
This module was modified from Makersnake's source code and re-released under the GNU-GPL-v3 license
Source code: https://github.com/MakerSnake/pyRockBlock

Modifications Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''
   
import glob
import signal
import struct
import sys
import time
import rospy

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
    def rockBlockRxFailed(self,mo_msg):pass
    def rockBlockRxReceived(self,mtmsn,data):pass
    def rockBlockRxMessageQueue(self,count):pass
     
    #MO
    def rockBlockTxStarted(self):pass
    def rockBlockTxFailed(self, momsg):pass
    def rockBlockTxSuccess(self,momsn, momsg):pass
    def rockBlockTxBlankMsg(self):pass
    
class rockBlockException(Exception):
    pass
    
class rockBlock(object):
    
    IRIDIUM_EPOCH = 1399818235000 # May 11, 2014, at 14:23:55 (This will be 're-epoched' every couple of years!)
        
    def __init__(self, portId, callback, client_serial):
        '''
        Initialize serial connection to Rockblock. If unable to connect to Rockblock, exception will be raised
        '''
        self.s = None
        self.portId = portId
        self.callback = callback
        self.autoSession = True # When True, we'll automatically initiate additional sessions if more messages to download

        self.mo_msg = "" # MO msg
        self.client_serial = client_serial # Client Rockblock serial no (to send to another Rockblock)
        
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
     
    
    def messageCheck(self, momsg):
        '''Check SBD mailbox for incoming MT msgs, and send MO msgs if MO buffer is not empty'''
        self._ensureConnectionStatus()
        if(self.callback != None and callable(self.callback.rockBlockRxStarted) ):
            self.callback.rockBlockRxStarted()
        # Clear any failed MO msgs from previous attempts
        self._clearMoBuffer()
        self.mo_msg = ""
        # If there is an MO msg waiting to be sent, queue it
        have_queued_msg = False
        if momsg:
            self.mo_msg = momsg
            if self._queueMessage():
                have_queued_msg = True # msg was successfully queued
        if( self._attemptConnection() and self._attemptSession(have_queued_msg) ):
            return True
        else:
            if(self.callback != None and callable(self.callback.rockBlockRxFailed) ):
                if have_queued_msg:
                    self.callback.rockBlockRxFailed(self.mo_msg)
                else:
                    self.callback.rockBlockRxFailed(" ")
                
        
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
        '''Send an MO msg, and return True/False depending on whether attempt was successful'''
        self._ensureConnectionStatus()

        if(self.callback != None and callable(self.callback.rockBlockTxStarted) ):
            self.callback.rockBlockTxStarted()

        self.mo_msg = msg

        if( self._queueMessage() and self._attemptConnection()  ):
            # Try a max of 3 unsuccessful times before giving up permanently
            SESSION_DELAY = 1
            SESSION_ATTEMPTS = 3
            while(True):
                SESSION_ATTEMPTS = SESSION_ATTEMPTS - 1
                if(SESSION_ATTEMPTS == 0):
                    break
                if( self._attemptSession(True) ):
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
    def _queueMessage(self):
        '''Prepare a Mobile-Originated (MO) msg'''
        self._ensureConnectionStatus()

        rospy.loginfo("Inserting MO msg: " + self.mo_msg)

        # Pack prefix to send to another Rockblock. To do: Turn this functionality on and off
        pre_1 = (b'RB',)
        s1 = struct.Struct('2s')
        packed_1 = s1.pack(*pre_1)
        pre_2 = (self.client_serial,)
        s2 = struct.Struct('> I')
        packed_2 = s2.pack(*pre_2)
        packed_2 = packed_2[1:] # Rock Seven insists that serial no is packed into 3 bytes, big endian
        
        # Pack regular payload. To do: Have different formats for regular/nonregular payload
        li = self.mo_msg.split(" ")
        li = list(map(int, li)) # Convert all str to int
        i = 0
        struct_cmd = '>' # Standardize all to same endianess (easier to unpack)
        while i < len(li):
            struct_cmd = struct_cmd + ' H'
            i = i + 1
        s3 = struct.Struct(struct_cmd)
        packed_data = s3.pack(*li)
        
        command = "AT+SBDWB=" + str(s1.size + s2.size-1 + s3.size)
        self.s.write((command + "\r").encode())
        
        if(self.s.readline().strip().decode() == command):
            if(self.s.readline().strip().decode() == "READY"):
                self.s.write(packed_1 + packed_2 + packed_data)
                # Calculate checksum. Checksum is least significant 2-bytes of the summation of the entire SBD
                # message. The high order byte must be sent first. For example if the FA were to send the
                # word “hello” encoded in ASCII to the ISU the binary stream would be hex 68 65 6c 6c 6f 02 14.
                # Updated checksum calculation formula from https://stackoverflow.com/questions/46813077/2-byte-checksum-in-python-for-iridium-sbd
                # >> 8 to get higher order byte, see https://stackoverflow.com/questions/19153363/what-does-hibyte-value-8-meaning
                # & 0xFF to get lower order byte, see https://oscarliang.com/what-s-the-use-of-and-0xff-in-programming-c-plus-p/
                # Assumption: Checksum not more than 16 bits (2 bytes), or a value of 65535 (always true for msgs < 340 characters)
                checksum = sum(packed_1 + packed_2 + packed_data)
                a = checksum >> 8
                b = checksum & 0xFF
                checksum = bytes([a,b])
                self.s.write(checksum)
                self.s.readline().strip()  #BLANK
                result = False
                queuestatus = self.s.readline().strip().decode()
                if queuestatus == "0":
                    rospy.loginfo("MO msg queue success")
                    result = True
                elif queuestatus == "1":
                    rospy.logwarn("MO msg write timeout")
                elif queuestatus == "2":
                    rospy.logwarn("MO msg corrupted")
                elif queuestatus == "3":
                    rospy.logwarn("MO msg too big")
                self.s.readline().strip()  #BLANK
                self.s.readline().strip() #OK
                return result
        return False
    
    
    def _configurePort(self):
        '''Initial setup of the serial port when opening connection to Rockblock'''
        if( self._enableEcho() and self._disableFlowControl and self._disableRingAlerts() and self.ping() ):
            return True
        else:
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
                 
                 
    def _attemptSession(self, have_queued_msg):
        '''
        Try to establish an Iridium SBD session and perform mailbox check. Works for both MO and MT msgs
        have_queued_msg determines whether there are MO msgs (that are successfully queued) waiting to be sent
        '''

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
                    if(moStatus <= 4 and have_queued_msg):
                        self._clearMoBuffer()
                        if(self.callback != None and callable(self.callback.rockBlockTxSuccess) ):   
                            self.callback.rockBlockTxSuccess( moMsn, self.mo_msg )
                        pass
                    elif (moStatus <= 4 and not have_queued_msg):
                        if(self.callback != None and callable(self.callback.rockBlockTxSuccess) ):   
                            self.callback.rockBlockTxBlankMsg()
                        pass
                    else:
                        if(self.callback != None and callable(self.callback.rockBlockTxFailed) ): 
                            self.callback.rockBlockTxFailed(self.mo_msg)
                    
                    # SBD message successfully received from the GSS.
                    if(mtStatus == 1 and mtLength > 0): 
                        self._processMtMessage(mtMsn)
                    
                    # AUTOGET NEXT MESSAGE
                    if(self.callback != None and callable(self.callback.rockBlockRxMessageQueue) ): 
                        self.callback.rockBlockRxMessageQueue(mtQueued)
                    
                    #There are additional MT messages to queued to download
                    if(mtQueued > 0 and self.autoSession == True):
                        self.mo_msg = "" # Clear MO buffer to avoid sending same MO msg twice
                        self._attemptSession(False)
                    
                    if(moStatus <= 4):                     
                        return True

        return False
     
        
    def _attemptConnection(self):
        '''Ensure valid network time and sufficient signal strength'''
        self._ensureConnectionStatus()

        TIME_ATTEMPTS = 20
        TIME_DELAY = 1
       
        SIGNAL_ATTEMPTS = 5
        RESCAN_DELAY = 1
        SIGNAL_THRESHOLD = 2
        
        # Wait for valid Network Time
        while True:
            if(TIME_ATTEMPTS == 0):
                rospy.logerr("No Iridium Network Service!")
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
        command = "AT+SBDRT"
        self.s.write((command + "\r").encode())
        self.s.readline() # AT+SBDRT
        self.s.readline() # SBDRT: 
        response = self.s.readline() # Actual text msg
        rospy.loginfo(response)
        response = response.decode()

        if( response == "OK" ):
            # Blank msg
            rospy.logwarn("No message content.. strange!")
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