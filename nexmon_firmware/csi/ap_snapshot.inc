#pragma NEXMON targetregion "patch"

/* Copy PHY tables under native MAC/PHY protection; export only from RAM. */
#define AP_CSI_PENDING 0xba0
#define AP_CSI_TSF_LOW 0xba9
#define AP_CSI_TSF_HIGH 0xbaa
#define AP_CSI_SERIAL 0xba4
#define AP_CSI_SOURCE 0xbb0
#define AP_CSI_SEQUENCE 0xbb3
#define AP_CSI_CHANSPEC 0xbb4
#define AP_CSI_PHY 0xbb5
#define AP_CSI_RX_GENERATION 0xbac
#define AP_CSI_CAPTURE_GENERATION 0xbab
#define AP_CSI_MAX_AGE_US 20000
#define AP_CSI_MAX_COPY_US 8000
#define AP_CSI_ARM_READY 0xba3
#define AP_CSI_COLLECT 0xb80
#define AP_CSI_COREMASK 0xb82
#define AP_CSI_NSSMASK 0xb81
#define AP_CSI_TONES 256
#define AP_CSI_STREAMS 16
#define AP_CSI_IDLE_MS 5
#define AP_CSI_DRAIN_MS 1
#define AP_CSI_WIRE_FLAG 0x80

struct ap_csi_trailer {
    uint8 magic[4];
    uint8 version;
    uint8 nsts;
    uint8 coremask;
    uint8 packet_bw;
    uint32 tsf;
    uint16 serial;
    uint16 sequence_control;
    uint16 phy[6];
    uint16 read_us;
    uint8 valid_tx_mask;
    uint8 reserved;
} __attribute__((packed));

struct ap_csi_snapshot {
    uint32 values[AP_CSI_STREAMS][AP_CSI_TONES];
    struct ap_csi_trailer trailer;
    uint8 source[6];
    int8 rssi;
    uint8 state;                 /* 0 free, 2 immutable/export */
    uint16 chanspec;
    uint16 tones;
    uint16 export_stream;
};

static struct ap_csi_snapshot *ap_csi;
static void *ap_csi_timer;
static uint8 ap_csi_timer_armed;
static uint32 ap_csi_unclaimed_since;
uint16 csi_diagnostic_packets = 0;
uint16 csi_diagnostic_alloc_failures = 0;
uint16 csi_diagnostic_xmit_failures = 0;
uint16 csi_diagnostic_incomplete = 0;
uint16 csi_diagnostic_snapshots = 0;
uint16 csi_diagnostic_max_read_us = 0;

static uint16
ap_csi_tones(uint16 chanspec)
{
    switch ((chanspec >> 11) & 7) {
        case 2: return 64;
        case 3: return 128;
        case 4: return 256;
        default: return 0;
    }
}

static int
ap_csi_carrier_valid(uint16 raw_index)
{
    uint16 n = ap_csi->tones;
    uint16 width_shift = ap_csi->trailer.packet_bw == 80 ? 2 :
                         (ap_csi->trailer.packet_bw == 40 ? 1 : 0);
    uint16 width = 1 << width_shift;
    uint16 primary = (ap_csi->chanspec >> 8) & 7;
    uint16 start;
    uint16 index = (raw_index + n / 2) & (n - 1);
    int offset, maximum, minimum;
    if (!width || width > n / 64 || primary >= n / 64) {
        return 0;
    }
    start = (primary >> width_shift) << (width_shift + 6);
    if (index < start || index >= start + width * 64) {
        return 0;
    }
    offset = (int)index - start - width * 32;
    if (offset < 0) {
        offset = -offset;
    }
    maximum = width == 4 ? 122 : (width == 2 ? 58 :
              ((ap_csi->trailer.phy[0] & 3) == 1 ? 26 : 28));
    minimum = width == 1 ? 1 : 2;
    return offset >= minimum && offset <= maximum;
}

static void
ap_csi_arm_timer(struct wlc_hw_info *hw, uint16 ms)
{
    if (!ap_csi_timer_armed) {
        ap_csi_timer_armed = 1;
        wl_add_timer(hw->wlc->wl, ap_csi_timer, ms, 0);
    }
}

static void
ap_csi_release(struct wlc_hw_info *hw)
{
    if (ap_csi) {
        ap_csi->state = 0;
    }
    ap_csi_unclaimed_since = 0;
    if (hw->up) {
        wlc_bmac_write_shm(hw, AP_CSI_PENDING * 2, 0);
    }
    if (hw->up && wlc_bmac_read_shm(hw, AP_CSI_COLLECT * 2)) {
        wlc_bmac_write_shm(hw, AP_CSI_ARM_READY * 2, 1);
    }
}


static void
ap_csi_timer_callback(void *arg)
{
    struct wlc_hw_info *hw = arg;
    struct wl_info *wl = hw->wlc->wl;
    uint16 delay = AP_CSI_IDLE_MS;
    uint32 now;
    ap_csi_timer_armed = 0;
    if (!hw->up || !ap_csi ||
        !wlc_bmac_read_shm(hw, AP_CSI_COLLECT * 2)) {
        return;
    }
    now = hw->regs->tsf_timerlow;
    if (ap_csi->state == 2) {
        uint16 index = ap_csi->export_stream;
        uint16 config = ((index & 3) << 8) | ((index >> 2) << 11) |
                        AP_CSI_WIRE_FLAG;
        uint16 length = sizeof(struct csi_udp_frame) +
                        ap_csi->tones * sizeof(uint32) +
                        sizeof(struct ap_csi_trailer);
        struct sk_buff *packet = pkt_buf_get_skb(wl->wlc->osh, length);
        if (!packet) {
            csi_diagnostic_alloc_failures++;
            ap_csi_release(hw);
            goto rearm;
        }
        struct csi_udp_frame *frame = (void *) packet->data;
        frame->kk1 = 0x1111;
        frame->rssi = ap_csi->rssi;
        frame->fc = 0x88;
        memcpy(frame->SrcMac, ap_csi->source, sizeof(frame->SrcMac));
        frame->seqCnt = ap_csi->trailer.sequence_control;
        frame->csiconf = config;
        frame->chanspec = ap_csi->chanspec;
        frame->chip = NEXMON_CHIP;
        uint16 carrier;
        for (carrier = 0; carrier < ap_csi->tones; carrier++) {
            frame->csi_values[carrier] = ap_csi_carrier_valid(carrier) ?
                                        ap_csi->values[index][carrier] : 0;
        }
        memcpy(&frame->csi_values[ap_csi->tones], &ap_csi->trailer,
               sizeof(ap_csi->trailer));
        packet->len = length;
        skb_pull(packet, sizeof(struct ethernet_ip_udp_header));
        prepend_ethernet_ipv4_udp_header(packet);
        if (wl->dev->chained->funcs->xmit(wl->dev, wl->dev->chained, packet)) {
            csi_diagnostic_xmit_failures++;
            ap_csi_release(hw);
            goto rearm;
        }
        csi_diagnostic_packets++;
        if (++ap_csi->export_stream == AP_CSI_STREAMS) {
            ap_csi_release(hw);
        } else {
            delay = AP_CSI_DRAIN_MS;
        }
    } else if (!wlc_bmac_read_shm(hw, AP_CSI_ARM_READY * 2)) {
        /* A missed RX callback must not leave a stale request wedged. */
        if (!ap_csi_unclaimed_since) {
            ap_csi_unclaimed_since = now;
        } else if ((uint32)(now - ap_csi_unclaimed_since) >= AP_CSI_MAX_AGE_US) {
            csi_diagnostic_incomplete++;
            ap_csi_release(hw);
        }
    } else {
        ap_csi_unclaimed_since = 0;
    }
rearm:
    ap_csi_arm_timer(hw, delay);
}

/* Called with collection disabled, before the enable word is published. */
int
ap_csi_configure(struct wlc_hw_info *hw)
{
    if (ap_csi_timer_armed) {
        wl_del_timer(hw->wlc->wl, ap_csi_timer);
        ap_csi_timer_armed = 0;
    }
    if (!ap_csi) {
        ap_csi = mallocz(sizeof(*ap_csi));
    }
    if (!ap_csi_timer) {
        ap_csi_timer = wl_init_timer(hw->wlc->wl, ap_csi_timer_callback, hw,
                                    "ap_csi_snapshot");
    }
    if (!ap_csi || !ap_csi_timer) {
        csi_diagnostic_alloc_failures++;
        return 0;
    }
    ap_csi->state = 0;
    ap_csi_unclaimed_since = 0;
    wlc_bmac_write_shm(hw, AP_CSI_PENDING * 2, 0);
    wlc_bmac_write_shm(hw, AP_CSI_ARM_READY * 2, 1);
    return 1;
}

void
ap_csi_start_timer(struct wlc_hw_info *hw)
{
    if (ap_csi && ap_csi_timer && hw->up &&
        wlc_bmac_read_shm(hw, AP_CSI_COLLECT * 2)) {
        ap_csi_arm_timer(hw, AP_CSI_IDLE_MS);
    }
}


/* The DMA header can omit PHY status on later aggregate members. RSSI must
 * come from this snapshot's latched PPDU, rather than that callback's header. */
static int8
ap_csi_packet_rssi(struct wlc_hw_info *hw)
{
    struct wlc_d11rxhdr header;
    uint16 i;
    for (i = 0; i < sizeof(header); i++) ((uint8 *)&header)[i] = 0;
    memcpy(&header.rxhdr.PhyRxStatus_0, ap_csi->trailer.phy, 12);
    header.rxhdr.RxStatus2 = 0x100; /* Valid PHY status. */
    header.rxhdr.RxChan = ap_csi->chanspec;
    wlc_phy_rssi_compute(hw->band->pi, &header);
    return header.rssi;
}

typedef char ap_csi_rssi_abi_check[
    __builtin_offsetof(struct wlc_d11rxhdr, rssi) == 40 ? 1 : -1];

/* Release PHY protection before resuming the MAC on every exit. */
static void
ap_csi_capture(struct wlc_hw_info *hw)
{
    uint16 generation, captured, nsts, coremask, nssmask, tones, word, tx, core;
    uint32 started, elapsed;
    int valid = 0;
    if (!hw->up || !ap_csi || ap_csi->state ||
        !wlc_bmac_read_shm(hw, AP_CSI_COLLECT * 2) ||
        !wlc_bmac_read_shm(hw, AP_CSI_PENDING * 2)) {
        return;
    }
    for (word = 0; word < AP_CSI_STREAMS * AP_CSI_TONES; word++) {
        ((uint32 *)ap_csi->values)[word] = 0;
    }
    started = hw->regs->tsf_timerlow;
    wlc_bmac_write_shm(hw, AP_CSI_ARM_READY * 2, 0);
    wlc_bmac_suspend_mac_and_wait(hw);
    // Any descriptor update already in flight finishes before native suspend.
    captured = wlc_bmac_read_shm(hw, AP_CSI_CAPTURE_GENERATION * 2);
    generation = wlc_bmac_read_shm(hw, AP_CSI_RX_GENERATION * 2);
    if ((hw->regs->maccontrol & 1) || !(hw->regs->macintstatus & 1) ||
        generation != captured || !wlc_bmac_read_shm(hw, AP_CSI_PENDING * 2)) {
        goto resume_mac;
    }
    ap_csi->chanspec = wlc_bmac_read_shm(hw, AP_CSI_CHANSPEC * 2);
    tones = ap_csi_tones(ap_csi->chanspec);
    nsts = wlc_bmac_read_shm(hw, 0xba7 * 2);
    coremask = wlc_bmac_read_shm(hw, AP_CSI_COREMASK * 2) & 15;
    nssmask = wlc_bmac_read_shm(hw, AP_CSI_NSSMASK * 2) & 15;
    if (!tones || !nsts || nsts > 4 || !coremask || !nssmask) {
        goto resume_mac;
    }
    ap_csi->tones = tones;
    memcpy(ap_csi->trailer.magic, "WSNP", 4);
    ap_csi->trailer.version = 2;
    ap_csi->trailer.nsts = nsts;
    ap_csi->trailer.coremask = coremask;
    ap_csi->trailer.valid_tx_mask = nssmask & ((1 << nsts) - 1);
    ap_csi->trailer.reserved = 0;
    ap_csi->trailer.packet_bw = wlc_bmac_read_shm(hw, 0xba8 * 2);
    if (ap_csi->trailer.packet_bw != 20 && ap_csi->trailer.packet_bw != 40 &&
        ap_csi->trailer.packet_bw != 80) {
        goto resume_mac;
    }
    ap_csi->trailer.serial = wlc_bmac_read_shm(hw, AP_CSI_SERIAL * 2);
    ap_csi->trailer.tsf = wlc_bmac_read_shm(hw, AP_CSI_TSF_LOW * 2) |
                       ((uint32)wlc_bmac_read_shm(hw, AP_CSI_TSF_HIGH * 2) << 16);
    ap_csi->trailer.sequence_control = wlc_bmac_read_shm(hw, AP_CSI_SEQUENCE * 2);
    for (word = 0; word < 6; word++) {
        ap_csi->trailer.phy[word] = wlc_bmac_read_shm(hw, (AP_CSI_PHY + word) * 2);
    }
    for (word = 0; word < 3; word++) {
        uint16 address = wlc_bmac_read_shm(hw, (AP_CSI_SOURCE + word) * 2);
        ap_csi->source[word * 2] = address & 255;
        ap_csi->source[word * 2 + 1] = address >> 8;
    }
    if ((uint32)(hw->regs->tsf_timerlow - ap_csi->trailer.tsf) >= AP_CSI_MAX_AGE_US) {
        goto resume_mac;
    }
    wlc_phyreg_enter(hw->band->pi);
    wlc_phy_stay_in_carriersearch_acphy(hw->band->pi, 1);
    for (core = 0; core < 4; core++) {
        if (!(coremask & (1 << core))) continue;
        for (tx = 0; tx < nsts; tx++) {
            if (!(nssmask & (1 << tx))) continue;
            wlc_phy_table_read_acphy(hw->band->pi, 73 + core * 32, tones,
                                    tx * 256, 32, ap_csi->values[tx * 4 + core]);
            if ((uint32)(hw->regs->tsf_timerlow - started) >= AP_CSI_MAX_COPY_US) {
                goto resume_phy;
            }
        }
    }
    valid = 1;
resume_phy:
    wlc_phy_stay_in_carriersearch_acphy(hw->band->pi, 0);
    wlc_phyreg_exit(hw->band->pi);
resume_mac:
    wlc_bmac_enable_mac(hw);
    elapsed = hw->regs->tsf_timerlow - started;
    if (!valid || elapsed >= AP_CSI_MAX_COPY_US) goto rejected;
    ap_csi->rssi = ap_csi_packet_rssi(hw);
    ap_csi->trailer.read_us = elapsed;
    wlc_bmac_write_shm(hw, AP_CSI_PENDING * 2, 0);
    ap_csi->export_stream = 0;
    ap_csi->state = 2;
    csi_diagnostic_snapshots++;
    if (elapsed > csi_diagnostic_max_read_us) csi_diagnostic_max_read_us = elapsed;
    ap_csi_arm_timer(hw, AP_CSI_DRAIN_MS);
    return;
rejected:
    csi_diagnostic_incomplete++;
    ap_csi_release(hw);
}
