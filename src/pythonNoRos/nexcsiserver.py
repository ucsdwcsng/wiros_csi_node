import socket
import struct
import subprocess
import os
import json
import datetime
import signal
import sys
import random
import string
import ast
from paho.mqtt import client as mqtt_client
from dotenv import load_dotenv
from typing import List, Tuple
import numpy as np

# Constants
CSI_BUF_SIZE = 16384
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
bw = 20
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

# Define masks (based on C++ code)
E_MASK = 0x3F  # 6-bit exponent (bits [0:5])
R_MANT_MASK = 0x1FFC0000  # Real mantissa (bits [18:28])
I_MANT_MASK = 0x00007FC0  # Imag mantissa (bits [6:16])
R_SIGN_MASK = 0x20000000  # Real sign (bit 29)
I_SIGN_MASK = 0x00020000  # Imag sign (bit 17)
COUNT_MASK = 0x400  # For mantissa normalization check (bit 10)
MANT_MASK = 0x3FF  # Lower 10 bits of mantissa


def decode_csi(csi_data, n_sub):
    csi = struct.unpack(f"<{n_sub}I", csi_data)
    csi_r_buf = []
    csi_i_buf = []

    for i in range(n_sub):
        c = csi[i]

        # Extract exponent and bias it
        exp = (c & E_MASK) - 31 + 1023
        r_exp = exp
        i_exp = exp

        # Extract mantissas
        r_mant = (c & R_MANT_MASK) >> 18
        i_mant = (c & I_MANT_MASK) >> 6

        # Normalize real mantissa
        e_shift = 0
        while not (r_mant & COUNT_MASK) and e_shift < 10:
            r_mant <<= 1
            e_shift += 1
        if e_shift == 10:
            r_exp = 1023
            r_mant = 0
        else:
            r_exp -= e_shift

        # Normalize imag mantissa
        e_shift = 0
        while not (i_mant & COUNT_MASK) and e_shift < 10:
            i_mant <<= 1
            e_shift += 1
        if e_shift == 10:
            i_exp = 1023
            i_mant = 0
        else:
            i_exp -= e_shift

        # Compose double-precision floats
        c_r = ((c & R_SIGN_MASK) << 34) | ((r_mant & MANT_MASK) << 42) | (r_exp << 52)
        c_i = ((c & I_SIGN_MASK) << 46) | ((i_mant & MANT_MASK) << 42) | (i_exp << 52)

        # Convert binary representation to float
        csi_r_buf.append(struct.unpack("d", struct.pack("Q", c_r))[0])
        csi_i_buf.append(struct.unpack("d", struct.pack("Q", c_i))[0])

    return csi_r_buf, csi_i_buf


def string_to_bool(string):
    string = string.lower()
    if string == "true":
        return True
    elif string == "false":
        return False
    else:
        raise ValueError("Invalid input: string must be 'true' or 'false'")


# MQTT Configuration
load_dotenv()
address = os.getenv("MQTT_ENDPOINT")
mqtt_port = int(os.getenv("MQTT_PORT"))
client_id = "".join(random.choices((string.ascii_letters + string.digits), k=6))
CLIENT = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, "client")
topic = "/csi-ap2"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {rc}\n")

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.on_connect = on_connect
    client.connect(address, mqtt_port)
    return client


class MacFilter:
    def __init__(self, mac: List[int], length: int):
        self.mac = mac
        self.len = length

    def matches(self, other_mac: List[int]) -> bool:
        if self.len == 0:
            return True
        return self.mac[: self.len] == other_mac[: self.len]


class CsiInstance:
    def __init__(self):
        self.source_mac = [0] * 6
        self.seq_num = 0
        self.rssi = 0
        self.tx = 4
        self.rx = 4
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
        self.complex_csi = None  # Added for MATLAB-like complex representation


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


# Load configuration
with open("config.json", "r") as file:
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

    AP_LAT = float(config_data["aoa_info"][0]["AP_lat"])
    AP_LONG = float(config_data["aoa_info"][0]["AP_long"])
    AP_LAT1 = float(config_data["aoa_info"][0]["lat1"])
    AP_LONG1 = float(config_data["aoa_info"][0]["long1"])
    AP_LAT2 = float(config_data["aoa_info"][0]["lat2"])
    AP_LONG2 = float(config_data["aoa_info"][0]["long2"])

    filter = MacFilter(mac_filter, length)


def sh_exec_block(cmd: str) -> str:
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
        )
    except subprocess.TimeoutExpired:
        print("Process Timeout")
        print("Retrying...")
        return
    print(result.stdout)
    return result.stdout


def handle_shutdown(sig, frame):
    print("Shutting down.")
    sys.exit(0)


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


def set_mac_filter(filt: MacFilter) -> bool:
    global filter, use_software_mac_filter
    if filt.len < 0 or filt.len > 6:
        return True
    filter = filt
    print(f"Set MAC FILTER to {filt.mac}")
    if filt.len == 2:
        use_software_mac_filter = False
    return False


def reconfigure() -> str:
    global iface, tx_nss
    if ch >= 32:
        iface = "eth6"
        tx_nss = min(tx_nss, 4)
    else:
        iface = "eth5"
        tx_nss = min(tx_nss, 3)

    if filter.len > 1:
        configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/setup.sh {ch} {bw} 4 {filter.mac[0]:02x}:{filter.mac[1]:02x}:{filter.mac[2]:02x}:{filter.mac[3]:02x}:{filter.mac[4]:02x}:{filter.mac[5]:02x} 2>&1"
    else:
        configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/setup.sh {ch} {bw} 4 2>&1"

    print(configcmd)
    return sh_exec_block(configcmd)


def reconnect() -> str:
    global iface, tx_nss
    if ch >= 32:
        iface = "eth6"
        tx_nss = min(tx_nss, 4)
    else:
        iface = "eth5"
        tx_nss = min(tx_nss, 3)

    if filter.len > 1:
        configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/configcsi.sh {ch} {bw} 4 {filter.mac[0]:02x}:{filter.mac[1]:02x}:{filter.mac[2]:02x}:{filter.mac[3]:02x}:{filter.mac[4]:02x}:{filter.mac[5]:02x} 2>&1"
    else:
        configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/configcsi.sh {ch} {bw} 4 2>&1"

    print(configcmd)
    return sh_exec_block(configcmd)


def reload_router() -> str:
    configcmd = f"sshpass -p {rx_pass} ssh -o strictHostKeyChecking=no {rx_host}@{rx_ip} /jffs/csi/reload.sh 2>&1"
    print(configcmd)
    return sh_exec_block(configcmd)


def ifconfig() -> str:
    configcmd = f"sudo ifconfig eth0 192.168.48.100 netmask 255.255.255.0"
    print(configcmd)
    return sh_exec_block(configcmd)


def parse_csi(data: bytes, nbytes: int):
    global channel_current, last_seq

    if nbytes < 18:
        print(f"Packet too small: {nbytes} bytes")
        return

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

    if use_software_mac_filter and not filter.matches(rxframe.src_mac):
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
    out.stamp = datetime.datetime.now()

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

    # Calculate expected CSI data size (4x4 MIMO)
    expected_csi_size = n_sub * 4
    if len(data) < 18 + expected_csi_size:
        print(f"CSI data too small: {len(data)-18} bytes, expected {expected_csi_size}")
        return

    # Decode CSI data
    out.csi_r, out.csi_i = decode_csi(data[18 : 18 + expected_csi_size], n_sub)

    # Create complex CSI representation (similar to MATLAB)
    out.complex_csi = np.array(out.csi_r) + 1j * np.array(out.csi_i)

    # Check for new CSI data
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


def build_csi_matrix(csi_list: List[CsiInstance]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build a (n_sub, 4, 4) complex matrix from a list of CsiInstance
    """
    n_sub = csi_list[0].n_sub
    csi_r = np.zeros((n_sub, 4, 4))
    csi_i = np.zeros((n_sub, 4, 4))

    for csi in csi_list:
        csi_r[:, csi.tx, csi.rx] = csi.csi_r
        csi_i[:, csi.tx, csi.rx] = csi.csi_i

    return csi_r.flatten().tolist(), csi_i.flatten().tolist()


def publish_csi(channel_current: List[CsiInstance]):
    if len(channel_current) < 16:
        print(f"Not enough CSI streams yet: {len(channel_current)}")
        return

    print(f"Publishing full CSI matrix from {len(channel_current)} streams...")

    # Build matrix and flatten
    csi_r_flat, csi_i_flat = build_csi_matrix(channel_current)

    # Use metadata from the first stream
    meta = channel_current[0]

    # Create a dictionary with all CSI data
    csi_data = {
        "mac": ":".join(f"{x:02x}" for x in meta.source_mac),
        "rssi": meta.rssi,
        "channel": meta.channel,
        "bw": meta.bw,
        "n_sub": meta.n_sub,
        "n_rows": meta.n_rows,
        "n_cols": meta.n_cols,
        "ap_id": meta.ap_id,
        "mcs": meta.mcs,
        "rx_id": meta.rx_id,
        "stamp": meta.stamp.isoformat(),
        "csi_r": csi_r_flat,
        "csi_i": csi_i_flat,
    }
    fc = 0

    msg = f"MAC: {meta.source_mac}, RSSI: {meta.rssi}, Channel: {meta.channel}, BW: {meta.bw}, csi_i: {csi_i_flat}, csi_r: {csi_r_flat}, fc: {fc}, n_sub: {meta.n_sub}, tx: {4}, n_rows: {meta.n_rows}, n_cols:{meta.n_cols}, ap_id: {meta.ap_id}, mcs: {meta.mcs}, rx_id: {meta.rx_id}, stamp;{meta.stamp}, AP_location: {[AP_LAT, AP_LONG]}, AP_L1: {[AP_LAT1, AP_LONG1]}, AP_L2: {[AP_LAT2, AP_LONG2]}"

    # Convert to JSON and publish
    # msg = json.dumps(csi_data)
    CLIENT.publish(topic, msg)
    print(f"Published full CSI matrix for MAC: {csi_data['mac']}, RSSI: {meta.rssi}")


def main():
    global rx_ip, rx_pass, rx_host, ch, bw, beacon, tx_nss, iface, filter, use_tcp, no_config, lock_topic

    signal.signal(signal.SIGINT, handle_shutdown)

    iface = "eth0"

    if not no_config:
        print("Configuring Receiver...")
        reconfigure()

    print("Starting CSI collection")
    sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockfd.settimeout(15.0)
    sockfd.bind(("0.0.0.0", PORT))

    while True:
        try:
            data, addr = sockfd.recvfrom(CSI_BUF_SIZE)
            if data:
                parse_csi(data, len(data))
        except socket.timeout:
            ifconfig()
            print("Socket Timeout")
            reload_router()
            print("Reloading Router...")
            reconnect()
            print("Starting CSI collection")
            sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockfd.settimeout(15.0)
            sockfd.bind(("0.0.0.0", PORT))
            continue
        except Exception as e:
            print(f"Socket Error: {e}")
            continue


if __name__ == "__main__":
    CLIENT = connect_mqtt()
    CLIENT.loop_start()
    main()
