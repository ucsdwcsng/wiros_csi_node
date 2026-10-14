// Runs after stock receive-header DMA. ARM clears ARM_READY before claiming
// the latest eligible descriptor.
 xje SHM_CSI_COLLECT, 0, csi_filter_skip+
 xje CSI_ARM_READY, 0, csi_filter_skip+
 xjl [0x86e], 30, csi_filter_skip+
 jnzx 0, 0, [0x876], 0, csi_filter_skip+
 mov 0x310, SPARE1
 and [0x870], SPARE1, SPARE1
 xjne SPARE1, 0, csi_filter_skip+
 xje APPLY_PKT_FILTER, 0, csi_no_byte_filter+
 and [0x814], 0xff, SPARE1
 xjne SPARE1, PKT_FILTER_BYTE, csi_filter_skip+
csi_no_byte_filter:
 mov SPR_BASE5, CSI_BASE5
 mov N_CMP_SRC_MAC, SPARE2
 xje SPARE2, 0, csi_mac_match+
 mov CMP_SRC_MAC_0_0, SPR_BASE5
csi_mac_loop:
 mov [0,off5], SPARE1
 xjne SPARE1, [0x819], csi_next_mac+
 mov [1,off5], SPARE1
 xjne SPARE1, [0x81a], csi_next_mac+
 mov [2,off5], SPARE1
 xjne SPARE1, [0x81b], csi_next_mac+
 jmp csi_mac_match+
csi_next_mac:
 xadd SPR_BASE5, 3, SPR_BASE5
 xsub SPARE2, 1, SPARE2
 xjne SPARE2, 0, csi_mac_loop-
 jmp csi_filter_restore+
csi_mac_match:
 // PLCP occupies 0x811..0x813; the MAC header starts at 0x814.
 // Only SU VHT and ordinary HT MCS 0..31 have this table layout.
 and [0x870], 3, SPARE1
 xje SPARE1, 3, csi_vht+
 xje SPARE1, 2, csi_ht+
 xjne SPARE1, 1, csi_filter_restore+
 mov 1, CSI_NSTS
 mov 20, CSI_PACKET_BW
 jmp csi_mark_frame+
csi_vht:
 and [0x811], 0x3f0, SPARE1
 xje SPARE1, 0, csi_vht_su+
 xjne SPARE1, 0x3f0, csi_filter_restore+
csi_vht_su:
 sr [0x811], 10, SPARE1
 and SPARE1, 7, SPARE1
 xadd SPARE1, 1, SPARE1
 xjg SPARE1, 4, csi_filter_restore+
 mov SPARE1, CSI_NSTS
 and [0x811], 3, SPARE1
 xje SPARE1, 3, csi_filter_restore+
 mov 20, SPARE2
 sl SPARE2, SPARE1, CSI_PACKET_BW
 jmp csi_mark_frame+
csi_ht:
 and [0x811], 0x7f, SPARE1
 xjg SPARE1, 31, csi_filter_restore+
 sr SPARE1, 3, SPARE1
 xadd SPARE1, 1, SPARE1
 // HT STBC adds training streams.
 sr [0x812], 12, SPARE2
 and SPARE2, 3, SPARE2
 xadd SPARE1, SPARE2, SPARE1
 xjg SPARE1, 4, csi_filter_restore+
 mov SPARE1, CSI_NSTS
 mov 20, CSI_PACKET_BW
 jzx 0, 7, [0x811], 0, csi_mark_frame+
 mov 40, CSI_PACKET_BW
csi_mark_frame:
 calls csi_begin_snapshot
csi_filter_restore:
 mov CSI_BASE5, SPR_BASE5
csi_filter_skip:
