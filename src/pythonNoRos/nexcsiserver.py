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
import ast
from paho.mqtt import client as mqtt_client
import time
from typing import List, Tuple

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


address = "128.205.218.189"
mqtt_port = 1883
client_id = "".join(random.choices((string.ascii_letters + string.digits), k=6))
CLIENT = mqtt_client.Client("client")
topic = "/csi"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(address, mqtt_port)
    return client


# MAC address filter
class MacFilter:
    def __init__(self, mac: List[int], length: int):
        self.mac = mac
        self.len = length


# CSI instance
class CsiInstance:
    def __init__(self):
        self.source_mac = [0] * 6
        self.seq_num = 0
        self.rssi = 0
        self.tx = 0
        self.rx = 0
        self.channel = 0
        self.bw = 0
        self.n_sub = 0
        self.csi_r = []
        self.csi_i = []
        self.seq = 0
        self.fc = 0
        self.n_cols = 4
        self.n_rows = 4
        self.ap_id = 0
        self.mcs = 0
        self.rx_id = 0
        self.stamp = None


# CSI UDP frame
class CsiUdpFrame:
    def __init__(self):
        self.kk1 = 0
        self.id = 0
        self.rssi = 0
        self.fc = 0
        self.src_mac = [0] * 6
        self.seqCnt = 0
        self.csiconf = 0
        self.chanspec = 0
        self.chip = 0


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

    filter = MacFilter(mac_filter, length)

# Execute a shell command and return the output
def sh_exec_block(cmd: str) -> str:
    result = subprocess.run(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    return result.stdout


# Handle shutdown signal
def handle_shutdown(sig, frame):
    print("Shutting down.")
    sys.exit(0)


# Set channel and bandwidth
def set_chanspec(s_chan: int, s_bw: int) -> bool:
    global ch, bw
    if not (s_bw == -1 or s_bw == 20 or s_bw == 40 or s_bw == 80):
        return True
    if s_chan != -1 and s_chan != ch:
        ch = s_chan
        print(f"Setting CHANNEL to {ch}")
    if s_bw != -1 and s_bw != bw:
        bw = s_bw
        print(f"Setting BW to {bw}")
    return False


# Set MAC filter
def set_mac_filter(filt: MacFilter) -> bool:
    global filter, use_software_mac_filter
    if filt.len < 0 or filt.len > 6:
        return True
    filter = filt
    print(f"Set MAC FILTER to {filt.mac}")
    if filt.len == 2:
        use_software_mac_filter = False
    return False


# Reconfigure the router
def reconfigure() -> str:
    global iface, tx_nss
    if ch >= 32:
        iface = "eth6"
        tx_nss = min(tx_nss, 4)
    else:
        iface = "eth5"
        tx_nss = min(tx_nss, 3)

    if filter.len > 1:
        configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/setup.sh {ch} {bw} 4 {filter.mac[0]:02x}:{filter.mac[1]:02x}:00:00:00:00 2>&1"
    else:
        configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/setup.sh {ch} {bw} 4 2>&1"
    print(configcmd)
    return sh_exec_block(configcmd)


# Parse CSI data
def parse_csi(data: bytes, nbytes: int):
    global channel_current, last_seq
    rxframe = CsiUdpFrame()
    rxframe.kk1 = data[0]
    rxframe.id = data[1]
    rxframe.rssi = struct.unpack("b", data[2:3])[0]
    rxframe.fc = data[3]
    rxframe.src_mac = list(data[4:10])
    rxframe.seqCnt = struct.unpack("<H", data[10:12])[0]
    rxframe.csiconf = struct.unpack("<H", data[12:14])[0]
    rxframe.chanspec = struct.unpack("<H", data[14:16])[0]
    rxframe.chip = struct.unpack("<H", data[16:18])[0]

    if use_software_mac_filter and rxframe.src_mac != filter.mac:
        return

    out = CsiInstance()
    out.rssi = rxframe.rssi
    out.source_mac = rxframe.src_mac
    out.seq = rxframe.seqCnt
    out.fc = rxframe.fc
    out.tx = (rxframe.csiconf >> 11) & 0x3
    out.rx = (rxframe.csiconf >> 8) & 0x3
    out.bw = (rxframe.chanspec >> 11) & 0x07
    out.channel = rxframe.chanspec & 255
    out.n_rows = 4
    out.n_cols = 4
    out.ap_id = 0
    out.rx_id = rx_ip
    out.stamp = time.time_ns()

    if out.bw == 0x4:
        out.bw = 80
    elif out.bw == 0x3:
        out.bw = 40
    elif out.bw == 0x2:
        out.bw = 20
    else:
        print(f"Invalid Bandwidth received {out.bw}")
        return

    n_sub = int(out.bw * 3.2)
    out.n_sub = n_sub
    out.csi_r = [0.0] * n_sub
    out.csi_i = [0.0] * n_sub

    csi = struct.unpack(f"<{n_sub}I", data[18 : 18 + n_sub * 4])

    for i in range(n_sub):
        out.csi_r[i] = float(csi[i] & 0xFFFF)  # Simplified CSI decoding
        out.csi_i[i] = float((csi[i] >> 16) & 0xFFFF)

    new_csi = False
    for ch_it in channel_current:
        if ch_it.tx * 4 + ch_it.rx == out.tx * 4 + out.rx:
            new_csi = True
    if out.seq != last_seq and len(channel_current) > 0:
        new_csi = True

    if new_csi:
        publish_csi(channel_current)
        channel_current.clear()

    last_seq = out.seq
    channel_current.append(out)


# Publish CSI data (replace with your own logic)
def publish_csi(channel_current: List[CsiInstance]):
    print("Publishing CSI data...")
    for csi in channel_current:
        msg = f"MAC: {csi.source_mac}, RSSI: {csi.rssi}, Channel: {csi.channel}, BW: {csi.bw}, csi_i: {csi.csi_i}, csi_r: {csi.csi_r}, fc: {csi.fc}, n_sub: {csi.n_sub}, tx: {csi.tx}, n_rows: {csi.n_rows}, n_cols:{csi.n_cols}, ap_id: {csi.ap_id}, mcs: {csi.mcs}, rx_id: {csi.rx_id}, stamp:{csi.stamp}"
        CLIENT.publish(topic, msg)
        print(
            f"MAC: {csi.source_mac}, RSSI: {csi.rssi}, Channel: {csi.channel}, BW: {csi.bw}, csi_i: {csi.csi_i}, csi_r: {csi.csi_r}, fc: {csi.fc}, n_sub: {csi.n_sub}, tx: {csi.tx}, n_rows: {csi.n_rows}, n_cols:{csi.n_cols}, ap_id: {csi.ap_id}, mcs: {csi.mcs}, rx_id: {csi.rx_id}, stamp:{csi.stamp}"
        )


# Main function
def main():
    global rx_ip, rx_pass, rx_host, ch, bw, beacon, tx_nss, iface, filter, use_tcp, no_config, lock_topic

    # Handle shutdown signal
    signal.signal(signal.SIGINT, handle_shutdown)

    # Setup parameters (replace with your own configuration)
    iface = "eth0"

    # Configure the router
    if not no_config:
        print("Configuring Receiver...")
        reconfigure()

    # Start CSI collection
    print("Starting CSI collection")
    sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockfd.settimeout(1.0)
    sockfd.bind(("0.0.0.0", PORT))

    while True:
        try:
            data, addr = sockfd.recvfrom(CSI_BUF_SIZE)
            parse_csi(data, len(data))
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Socket Error: {e}")
            continue


if __name__ == "__main__":
    CLIENT = connect_mqtt()
    main()
