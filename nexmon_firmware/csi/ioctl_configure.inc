            if (len < sizeof(*params) || params->n_mac_addr > 4) {
                break;
            }
#if NEXMON_CHIP == CHIP_VER_BCM4366c0
            if (!wlc->hw->up) {
                break;
            }
            if (!params->csi_collect) {
                wlc_bmac_write_shm(wlc->hw, SHM_CSI_COLLECT * 2, 0);
                wlc_bmac_write_shm(wlc->hw, 0xba0 * 2, 0);
                ret = IOCTL_SUCCESS;
                break;
            }
            if (wlc_bmac_read_shm(wlc->hw, 0xba0 * 2) ||
                get_chanspec(wlc) != params->chanspec) {
                break;
            }
            uint16 cores = params->core_nss_mask & 15;
            uint16 streams = params->core_nss_mask >> 4;
            if (params->csi_collect &&
                ((cores != 1 && cores != 3 && cores != 7 && cores != 15) ||
                 (streams != 1 && streams != 3 && streams != 7 && streams != 15))) {
                break;
            }
            wlc_bmac_write_shm(wlc->hw, SHM_CSI_COLLECT * 2, 0);
            wlc_bmac_write_shm(wlc->hw, 0xba3 * 2, 0);
            if (params->csi_collect && !ap_csi_configure(wlc->hw)) {
                break;
            }
#endif
            // deactivate scanning