void
ap_csi_process_frame(struct sk_buff *packet, struct wlc_d11rxhdr *rxhdr,
                     struct wlc_hw_info *hw, int tsf)
{
    rxhdr->tsf_l = tsf;
    wlc_phy_rssi_compute(hw->band->pi, rxhdr);
    ap_csi_capture(hw);
    wlc_recv(hw->wlc, packet);
}

__attribute__((at(0x2456E8, "", CHIP_VER_BCM4366c0, FW_VER_10_10_122_20)))
__attribute__((naked))
void ap_csi_receive_prehook(void)
{
    asm("mov r2, r4\n"
        "ldr r0, [sp,0x8]\n"
        "bl ap_csi_process_frame\n"
        "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n");
}
