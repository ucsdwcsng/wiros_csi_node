import math
import struct
import time


HEADER_SIZE = 18
BANDWIDTHS = {2: 20, 3: 40, 4: 80}
E_MASK = 0x3F
R_MANT_MASK = 0x1FFC0000
I_MANT_MASK = 0x0001FFC0
R_SIGN_MASK = 0x20000000
I_SIGN_MASK = 0x00020000
SNAPSHOT_FLAG = 0x80
SNAPSHOT_TRAILER = struct.Struct("<4sBBBBIHH6HHBB")


def decode_value(value):
    """Decode sign/magnitude I/Q with a signed six-bit exponent."""
    exponent = value & E_MASK
    if exponent & 0x20:
        exponent -= 0x40
    real = (value & R_MANT_MASK) >> 18
    imag = (value & I_MANT_MASK) >> 6
    if value & R_SIGN_MASK:
        real = -real
    if value & I_SIGN_MASK:
        imag = -imag
    return math.ldexp(real, exponent), math.ldexp(imag, exponent)


def valid_subcarrier_ranges(bandwidth, packet_bandwidth, control_sideband, frame_type):
    """Occupied LTF tones in the final FFT-shifted radio-width vector."""
    groups = bandwidth // 20
    width = packet_bandwidth // 20
    if width not in (1, 2, 4) or width > groups or not 0 <= control_sideband < groups:
        raise ValueError("invalid snapshot subcarrier geometry")
    start = (control_sideband // width) * width * 64
    center = start + width * 32
    maximum = 122 if width == 4 else (58 if width == 2 else (26 if frame_type == 1 else 28))
    minimum = 1 if width == 1 else 2
    return [[center - maximum, center - minimum + 1],
            [center + minimum, center + maximum + 1]]


def parse_frame(data, receiver_id, ap_id):
    if len(data) < HEADER_SIZE:
        raise ValueError("incomplete CSI header")
    kk1, frame_id, rssi, frame_control = struct.unpack("<BBbB", data[:4])
    if (kk1, frame_id) != (0x11, 0x11):
        raise ValueError(
            f"invalid CSI header magic: 0x{kk1:02x} 0x{frame_id:02x}"
        )
    source_mac = data[4:10]
    sequence, csi_config, chanspec, chip = struct.unpack("<HHHH", data[10:HEADER_SIZE])
    bandwidth = BANDWIDTHS.get((chanspec >> 11) & 7)
    if bandwidth is None:
        raise ValueError("unsupported CSI bandwidth")

    subcarrier_count = int(bandwidth * 3.2)
    frame_size = HEADER_SIZE + subcarrier_count * 4
    if len(data) < frame_size:
        raise ValueError(f"incomplete CSI frame: {len(data)} of {frame_size} bytes")

    values = struct.unpack(
        f"<{subcarrier_count}I", data[HEADER_SIZE:frame_size]
    )
    decoded = [decode_value(value) for value in values]
    payload = {
        "schema_version": 2,
        "encoding": "bcm4366-signed-exponent-v1",
        "timestamp_ns": time.time_ns(),
        "receiver_id": receiver_id,
        "ap_id": ap_id,
        "source_mac": ":".join(f"{value:02x}" for value in source_mac),
        "sequence": sequence,
        "rssi": rssi,
        "channel": chanspec & 255,
        "bandwidth_mhz": bandwidth,
        "tx": (csi_config >> 11) & 3,
        "rx": (csi_config >> 8) & 3,
        "frame_control": frame_control,
        "frame_id": frame_id,
        "magic": kk1,
        "chip": chip,
        "n_subcarriers": subcarrier_count,
        "csi_r": [value[0] for value in decoded],
        "csi_i": [value[1] for value in decoded],
    }

    if csi_config & SNAPSHOT_FLAG:
        if csi_config & ~0x1B80 or len(data) != frame_size + SNAPSHOT_TRAILER.size:
            raise ValueError("invalid snapshot frame size or stream config")
        metadata = SNAPSHOT_TRAILER.unpack_from(data, frame_size)
        magic, version, nsts, coremask, packet_bw, tsf, serial, seq = metadata[:8]
        phy = metadata[8:14]
        read_us, txmask, reserved = metadata[14:]
        if (magic != b"WSNP" or version not in (1, 2) or reserved != 0
                or not 1 <= nsts <= 4 or not 1 <= coremask <= 15
                or packet_bw not in BANDWIDTHS.values() or packet_bw > bandwidth
                or seq != sequence or not txmask
                or txmask & ~((1 << nsts) - 1)):
            raise ValueError("invalid snapshot metadata")
        if (not txmask & (1 << payload["tx"])
                or not coremask & (1 << payload["rx"])) and any(values):
            raise ValueError("nonzero CSI in an unmeasured snapshot stream")
        sideband = (chanspec >> 8) & 7
        ranges = valid_subcarrier_ranges(bandwidth, packet_bw, sideband, phy[0] & 3)
        valid_carriers = {index for start, stop in ranges for index in range(start, stop)}
        if version == 2 and any(
                value for index, value in enumerate(values)
                if (index + subcarrier_count // 2) % subcarrier_count not in valid_carriers):
            raise ValueError("nonzero CSI in an unmeasured subcarrier")
        payload.update({
            "snapshot_protocol_version": version,
            "control_sideband": sideband,
            "primary_channel": (chanspec & 255) - (bandwidth // 20 - 1) * 2 + sideband * 4,
            "valid_subcarrier_ranges": ranges,
            "coherent_snapshot": True,
            "capture_id": f"{tsf:08x}:{serial:04x}",
            "capture_tsf_us": tsf,
            "snapshot_sequence": serial,
            "sequence_control": seq,
            "n_tx_observed": nsts,
            "valid_tx_mask": txmask,
            "valid_rx_mask": coremask,
            "packet_bandwidth_mhz": packet_bw,
            "phy_status": list(phy),
            "snapshot_read_us": read_us,
        })
    return payload
