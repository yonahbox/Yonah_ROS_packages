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
        self._recovery_inter = 3 # Extended interval
        self._link_state = 0 # -1 = link down, 0 = link initialised/in recovery mode, 1 = link up
        self._srtt = 0 # Smoothed round-trip-time (rtt)
        self._rttvar = 0 # rtt variance
        self._rto = 1 # "Retransmission" timeout. In our case it is the watchdog timeout value
        self._recovery_rto = 3 * self._recovery_inter # rto used during recovery phase
        self._clock_gran = 1 # Clock granularity (seconds)

    def calc_rto(self, sent_timestamp):
        '''Calculate Retransmission Timeout according to Jacobson's Algo (RFC6298)'''
        if self._link_state == -1:
            return
        recv_timestamp = int(time.time())
        r = recv_timestamp - sent_timestamp + self._inter # r = measured rtt
        if self._link_state == 0:
            self._rttvar = 0.5*r
            self._srtt = r
            self._link_state = 1
        else:
            # Use RFC recommended values of alpha = 0.125 and beta = 0.25. Numbers are hardcoded to save memory
            self._rttvar = 0.75 * self._rttvar + 0.25 * abs(self._srtt - r)
            self._srtt = 0.875 * self._srtt + 0.125 * r
        self._rto = self._srtt + max(self._clock_gran, 4 * self._rttvar)

    def reset_rto(self):
        '''Set RTO to recovery RTO. Used during bootup and link recovery'''
        self._rto = self._recovery_rto
        self._link_state = 0

    def set_link_state(self, state):
        if state == -1 or state == 0 or state == 1:
            self._link_state = state

    def set_interval(self, inter):
        self._inter = inter

    def set_recovery_interval_and_rto(self, extended_inter):
        self._recovery_inter = extended_inter
        self._recovery_rto = 3 * self._recovery_inter
    
    def get_rto(self):
        return self._rto

    def get_link_state(self):
        return self._link_state