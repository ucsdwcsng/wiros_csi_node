import socket
import struct
import subprocess
import re
import json
import time
import signal
import sys
import random
import string
from paho.mqtt import client as mqtt_client
import time
from typing import List, Tuple
import ast

# Constants
CSI_BUF_SIZE = 4096
PORT = 5500
PORT_TCP = 50005

# Global variables
channel_current = []
last_seq = 0
rx_ip = ""
rx_pass = "password"
rx_host = "admin"
use_tcp = False
no_config = False
hostname = ""
ch = 157
bw = 80
beacon = 200.0
iface = ""
tx_nss = 4
filter = []
use_software_mac_filter = False
lock_topic = ""
csi_buf = bytearray(CSI_BUF_SIZE)
csi_data = bytearray(CSI_BUF_SIZE)
csi_r_out = None
csi_i_out = None
csi_size = 0


def string_to_bool(string):
    string = string.lower()
    if string == "true":
        return True
    elif string == "false":
        return False
    else:
        raise ValueError("Invalid input: string must be 'true' or 'false'")


with open("src/pythonNoRos/config.json", "r") as file:
    config_data = json.load(file)

    rx_ip = config_data["login"][0]["host_ip"]
    rx_host = config_data["login"][0]["host"]
    rx_pass = config_data["login"][0]["host_passwd"]

    ch = int(config_data["channel_params"][0]["channel"])
    bw = int(config_data["channel_params"][0]["bw"])

    use_software_mac_filter = string_to_bool(
        config_data["packet_params"][0]["use_software_mac_filter"]
    )
    beacon = float(config_data["packet_params"][0]["beacon_rate"])
    tx_nss = int(config_data["packet_params"][0]["beacon_tx_nss"])

    use_tcp = string_to_bool(config_data["broadcast"][0]["tcp_forward"])

    mac_filter = ast.literal_eval(config_data["packet_params"][0]["mac_filter"])
    length = int(config_data["packet_params"][0]["length"])

    print(rx_ip)
    print(rx_host)
    print(rx_pass)
    print(ch)
    print(bw)
    print(use_software_mac_filter)
    print(beacon)
    print(tx_nss)
    print(use_tcp)
    print(mac_filter)
    print(length)
