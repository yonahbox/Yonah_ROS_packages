#!/usr/bin/env python3

'''
Dynamically calculate the watchdog countdown interval for switcher node

The interval calculation adopts Jacobson's Algorithm for TCP RTT calculation
as given by RFC6298: https://tools.ietf.org/html/rfc6298

The recovery behaviour adopts modified version of Eifel Algorithm
as given by RFC3522: https://tools.ietf.org/html/rfc3522
'''

# Copyright (C) 2021, Lau Yan Han and Yonah (yonahbox@gmail.com)

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import time

class dynamic_delay():
    '''Each link monitor in switcher should spawn an instance of this calculator'''
    def __init__(self):
        self._inter = 1 # Interval between sender's msgs
        self._extended_inter = 3 # Extended interval
        self._first_r = True # True = waiting for first incoming msg after startup/restoration of link
        self._srtt = 0 # Smoothed round-trip-time (rtt)
        self._rttvar = 0 # rtt variance
        self._rto = 1 # "Retransmission" timeout. In our case it is the watchdog timeout value
        self._recovery_rto = 1.5 * self._extended_inter # rto used during recovery phase
        self._clock_gran = 1 # Clock granularity (seconds)

    def calc_rto(self, sent_timestamp):
        '''Calculate Retransmission Timeout according to Jacobson's Algo (RFC6298)'''
        recv_timestamp = time.time().seconds
        r = recv_timestamp - sent_timestamp + self._inter # r = measured rtt
        if self._first_r:
            self._rttvar = 0.5*r
            self._srtt = r
            self._first_r = False
        else:
            # Use RFC recommended values of alpha = 0.125 and beta = 0.25. Numbers are hardcoded to save memory
            self._rttvar = 0.75 * self._rttvar + 0.25 * abs(self._srtt - r)
            self._srtt = 0.875 * self._srtt + 0.125 * r
        self._rto = self._srtt + max(self._clock_gran, 4 * self._rttvar)

    def set_initial_rto(self, t):
        '''Initialize RTO to t. Used during bootup or on restoration of link'''
        self._rto = t
        self._first_r = True

    def set_interval(self, inter):
        self._inter = inter

    def set_extended_interval(self, extended_inter):
        self._extended_inter = extended_inter
    
    def get_rto(self):
        return self._rto

