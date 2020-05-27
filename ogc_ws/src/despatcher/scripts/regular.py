#!/usr/bin/env python3

"""
regular.py: Module to handle regular payload logic

Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

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
"""

# The regular payload should comprise only of short integers/characters
# with the exception of the first (R msg prefix) and last (Unix timestamp)
# Everything is standardized to big endian to keep in line with Rock 7's requirements
struct_cmd = "> s H H H H H H H H H H H H H I"

class RegularPayloadException(Exception):
    def __init__(self, msg):
        self.msg = msg
        super().__init__(self.msg)

    def __str__(self):
        return self.msg

def get_compressed_len():
    '''Return length of compressed regular payload based on struct_cmd'''
    global struct_cmd
    cmd_breakdown = struct_cmd.split(" ")[1:]
    size = 0
    for i in cmd_breakdown:
        if i == 's' or i == 'B' or i == 'b':
            size = size + 1
        elif i == 'H' or i == 'h':
            size = size + 2
        elif i == "I" or i == 'i':
            size = size + 4
        else:
            raise RegularPayloadException\
                ("Forbidden value " + i + " in struct cmd. Allowed values: s, B, b, H, h, I, i")
    return size

def convert_regular_payload(mo_msg):
    '''Convert regular payload msg from string to list for easy struct packing'''
    li = mo_msg.split(" ")
    return [li[0].encode(),] + list(map(int, li[1:])) # Convert all str to int (except prefix)