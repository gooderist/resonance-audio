#!/usr/bin/python
import sys
import os
import array

filename = sys.argv[1]

# Convert the input file.wav into a txt file with a hex presentation of the binary contents
f = open(filename, 'rb')
file_data = f.read()
ints = array.array('B', file_data)
data_string = ','.join('0x%x' % value for value in ints)

text_file = open("HRIR_DATA.txt", "w")
text_file.write(data_string)
text_file.close()