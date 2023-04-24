import rosbag
import os
import sys

if len(sys.argv) < 2:
    print(f"Must provide a bag file as argument.")
    sys.exit()

if not os.path.isfile(sys.argv[1]):
    print(sys.argv[1], 'is not a file.')
    sys.exit()

f = rosbag.Bag(sys.argv[1], 'r')

topicname = '/csi' if len(sys.argv) < 3 else sys.argv[2]

channelcount = {}
transmitters = {}
devices = {}

for topic, msg, t in f.read_messages(topicname):
    tx = f"{msg.txmac[0]:x}:{msg.txmac[1]:x}:{msg.txmac[2]:x}:{msg.txmac[3]:x}:{msg.txmac[4]:x}:{msg.txmac[5]:x}"
    spec = f"{msg.chan}/{msg.bw}"
    rx = msg.rx_id

    if spec not in channelcount:
        channelcount[spec] = 1
    else:
        channelcount[spec] += 1

    if tx not in transmitters:
        transmitters[tx] = 1
    else:
        transmitters[tx] += 1

    if rx not in devices:
        devices[rx] = 1
    else:
        devices[rx] += 1

print(f"\n---Transmitters Detected---\nMAC\t\t\tCount")
for tx in transmitters:
    print(f"{tx}\t{transmitters[tx]}")

print(f"\n---CHANSPECs Detected---\nSpec\tCount")
for spec in channelcount:
    print(f"{spec}\t{channelcount[spec]}")

print(f"\n---Receiver Devices---\nID\t\tCount")
for rx in devices:
    print(f"{rx}\t{devices[rx]}")

print("")
