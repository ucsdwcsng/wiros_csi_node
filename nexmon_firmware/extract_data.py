#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 30 14:45:27 2021

@author: wcsng-ros
"""
import h5py as hio
import numpy as np
from os.path import join
import sys
import os
import subprocess
import struct
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--scale", help="Manual scaling value (don't set to use autoscale)", default=32)
parser.add_argument("-p", "--path", help="Path to pcap data", default = "")
args = parser.parse_args()
assert args.path != "", "Please enter the data path with -p"
if os.path.isdir(args.path):
   data_paths = [join(args.path, x) for x in os.listdir(args.path) if ".pcap" in x]
elif os.path.isfile(args.path):
   data_paths = [args.path]
else:
   print("Could not find",args.path)
   sys.exit()

pcap_reader = os.path.abspath(join(os.path.abspath(__file__), "..", "readpcap"))

def nexmon_channels_from_log(path):
   csi_out = {}
   rssi_out = {}
   times_out = {}
   chan_out = {}
   f = open(path, 'rb')
   f.seek(0,2)
   file_size = f.tell()
   f.seek(0,0)
   vers_num = struct.unpack('i', f.read(I32))[0]
   if vers_num > 10 or vers_num < 0:
      vers_num = 0
      f.seek(0,0)
   while file_size - f.tell() > 0:
      #first 6 bytes contain mac address
      mac_addr = []
      for i in range(6):
         mac_addr.append(struct.unpack('B', f.read(1))[0])
      cur_mac = tuple(mac_addr)
         
      n_ints = struct.unpack('i', f.read(I32))[0]
      #then same for n_rx , n_tx, n_sub
      n_rx = struct.unpack('i', f.read(I32))[0]
      n_tx = struct.unpack('i', f.read(I32))[0]
      n_sub = int(struct.unpack('i', f.read(I32))[0] / 2)
      #then 4 bytes for time, utime, rssi.
      time = struct.unpack('I', f.read(I32))[0]
      utime = struct.unpack('I', f.read(I32))[0]
      rssi = struct.unpack('i', f.read(I32))[0]
      if vers_num >= 1:
         chan = struct.unpack('I', f.read(I32))[0]
      else:
         chan = 0;
      #interpret as signed
      if rssi > 127:
         rssi -= 255;
      #print(f"{cur_mac}: {rssi}")
      #now read data and convert to numpy until there is no more.
      #data is interleaved int32 real and imaginary parts.
      csi_data = f.read(n_ints*I32)
      cur_frame = np.zeros(int(n_ints / 2), dtype=np.complex64)

      for sub in range(cur_frame.shape[0]):
         bctr = sub*8
         re = struct.unpack('i', csi_data[bctr:bctr+4])[0]
         im = struct.unpack('i', csi_data[bctr+4:bctr+8])[0]
         cur_frame[sub] = re + 1.0j*im

      frame_out = np.zeros((n_sub,n_rx,n_tx), dtype=np.complex64)
      for tx in range(n_tx):
         for rx in range(n_rx):
            nss_idx = n_sub*(rx + tx*n_rx)
            frame_out[:,rx,tx] = cur_frame[nss_idx:nss_idx+n_sub]
      mac = tuple(mac_addr)
      try:
         rssi_out[mac].append(rssi)
         times_out[mac].append(time)
         csi_out[mac].append(frame_out)
         chan_out[mac].append(chan)
      except:
         rssi_out[mac] = []
         times_out[mac] = []
         csi_out[mac] = []
         chan_out[mac] = []
         rssi_out[mac].append(rssi)
         times_out[mac].append(time)
         csi_out[mac].append(frame_out)
         chan_out[mac].append(chan)
      
   return csi_out, rssi_out, times_out, chan_out

def np_list_cat(l):
	return np.concatenate(tuple(x[np.newaxis, :] for x in l), axis=0)

for data_file in data_paths:
   f_out_name = data_file.replace(".pcap", ".h5")
   csi_file = data_file.replace(".pcap", ".csi")

   command = [pcap_reader, data_file, "-o", csi_file]
   if args.scale != 32:
      command.append("-s")
      command.append(str(args.scale))

   print(command)
   subprocess.call(command)

   cur_mac = tuple()
   I32 = 4
   csi, rssi, time, chan = nexmon_channels_from_log(csi_file)

   f_out = hio.File(f_out_name, 'w')

   macs = []
   for mac in csi:
      csi_mat = np_list_cat(csi[mac])
      rssi_mat = np.asarray(rssi[mac])
      time_mat = np.asarray(time[mac])
      mac_hex = ":".join("%02x" % x for x in mac)
      macs.append(mac_hex)
      f_out.create_dataset(mac_hex, data=csi_mat)
      f_out.create_dataset(mac_hex + "-rssi", data=rssi_mat)
      f_out.create_dataset(mac_hex + "-time", data=time_mat)

   f_out.close()
   print("Saved CSI to", f_out_name)

   subprocess.call(['rm', csi_file])
