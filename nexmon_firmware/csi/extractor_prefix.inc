#include <firmware_version.h>
#if NEXMON_CHIP == CHIP_VER_BCM4366c0
#pragma NEXMON targetregion "patch"
#include <wrapper.h>
#include <structs.h>
#include <patcher.h>
#include <helper.h>
extern void prepend_ethernet_ipv4_udp_header(struct sk_buff *p);
struct csi_udp_frame {
    struct ethernet_ip_udp_header hdrs;
    uint16 kk1;
    int8 rssi;
    uint8 fc;
    uint8 SrcMac[6];
    uint16 seqCnt, csiconf, chanspec, chip;
    uint32 csi_values[];
} __attribute__((packed));
#include "ap_snapshot.inc"
#else
