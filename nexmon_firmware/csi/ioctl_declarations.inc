#if NEXMON_CHIP == CHIP_VER_BCM4366c0
extern int ap_csi_configure(struct wlc_hw_info *hw);
extern void ap_csi_start_timer(struct wlc_hw_info *hw);
extern uint16 csi_diagnostic_packets;
extern uint16 csi_diagnostic_alloc_failures;
extern uint16 csi_diagnostic_xmit_failures;
extern uint16 csi_diagnostic_incomplete;
extern uint16 csi_diagnostic_snapshots;
extern uint16 csi_diagnostic_max_read_us;
#endif
