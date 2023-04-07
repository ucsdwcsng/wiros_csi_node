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

for topic, msg, t in f.read_messages(topicname):
    tx = f"{msg.txmac[0]:x}:{msg.txmac[1]:x}:{msg.txmac[2]:x}:{msg.txmac[3]:x}:{msg.txmac[4]:x}:{msg.txmac[5]:x}"
    spec = f"{msg.chan}/{msg.bw}"

    if spec not in channelcount:
        channelcount[spec] = 1
    else:
        channelcount[spec] += 1

    if tx not in transmitters:
        transmitters[tx] = 1
    else:
        transmitters[tx] += 1

print(f"\n---Transmitters Detected---\nMAC\t\t\tCount")
for tx in transmitters:
    print(f"{tx}\t{transmitters[tx]}")

print(f"\n---CHANSPECs Detected---\nSpec\tCount")
for spec in channelcount:
    print(f"{spec}\t{channelcount[spec]}")
print("")
