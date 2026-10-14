#!/usr/bin/env python3
"""Generate the AP snapshot overlay from pinned Nexmon CSI sources."""
import argparse
import difflib
from pathlib import Path
import subprocess
import tempfile

NEXMON_REVISION = "68720406f54ff16cad28fd3c51f2392aefad9fb7"
CSI_REVISION = "1232edf2cf1e7ae0561676c49e4dd4eb22c8a27e"
RESOURCES = Path(__file__).resolve().parent
UCODE_PATCH = "csi.ucode.bcm4366c0.10_10_122_20.patch"


def upstream_text(upstream, filename):
    return subprocess.check_output(
        ["git", "-C", str(upstream), "show", f"{CSI_REVISION}:{filename}"],
        text=True,
    )


def integrate(upstream, workdir):
    resources = RESOURCES
    def orig(filename):
        return upstream_text(upstream, filename)
    s = orig('src/csi_extractor.c')
    prefix = resources.joinpath('extractor_prefix.inc').read_text()
    headers = s[s.index('struct d11rxhdr {'):s.index('struct csi_udp_frame {')]
    headers = '#define WL_RSSI_ANT_MAX 4\n' + headers
    hook = resources.joinpath('receive_hook.inc').read_text()
    prefix = prefix.replace('#include "ap_snapshot.inc"\n#else\n', headers + '#include "ap_snapshot.inc"\n' + hook + '\n#else\n')
    workdir.joinpath('src/csi_extractor.c').write_text(prefix + s + '\n#endif\n')
    workdir.joinpath('src/ap_snapshot.inc').write_text(resources.joinpath('ap_snapshot.inc').read_text())
    s = orig('src/ioctl.c')
    needle = 'int \nwlc_ioctl_hook('
    pre = resources.joinpath('ioctl_declarations.inc').read_text()
    s = s.replace(needle, pre + needle, 1)
    s = s.replace('            // deactivate scanning', resources.joinpath('ioctl_configure.inc').read_text(), 1)
    s = s.replace('            set_chanspec(wlc, params->chanspec);', '            if (get_chanspec(wlc) != params->chanspec) {\n                set_chanspec(wlc, params->chanspec);\n            }', 1)
    s = s.replace('                wlc_bmac_write_shm(wlc->hw, SHM_CSI_COLLECT * 2, params->csi_collect);', '', 1)
    needle = '                wlc_bmac_write_shm(wlc->hw, FIFODELAY * 2, params->delay);'
    s = s.replace(needle, needle + '\n                wlc_bmac_write_shm(wlc->hw, SHM_CSI_COLLECT * 2, params->csi_collect);\n#if NEXMON_CHIP == CHIP_VER_BCM4366c0\n                ap_csi_start_timer(wlc->hw);\n#endif', 1)
    needle = '                *(uint16 *) arg = wlc_bmac_read_shm(wlc->hw, SHM_CSI_COLLECT * 2);'
    s = s.replace(needle, needle + '\n#if NEXMON_CHIP == CHIP_VER_BCM4366c0\n                if (len >= 16) {\n                    uint16 *out = (uint16 *)arg;\n                    out[1] = 0x5351; /* Snapshot protocol identifier. */\n                    out[2] = csi_diagnostic_snapshots;\n                    out[3] = csi_diagnostic_packets;\n                    out[4] = csi_diagnostic_incomplete;\n                    out[5] = csi_diagnostic_alloc_failures;\n                    out[6] = csi_diagnostic_xmit_failures;\n                    out[7] = csi_diagnostic_max_read_us;\n                }\n#endif', 1)
    workdir.joinpath('src/ioctl.c').write_text(s)
    s = orig('src/version.c').replace('char version[] = "10.10.122.20 (nexmon.org/csi: " GIT_VERSION "-" BUILD_NUMBER ")\\n";', 'char version[] = "10.10.122.20 (nexmon.org/csi: ap-snapshot-1.0)\\n";')
    workdir.joinpath('src/version.c').write_text(s)
    s = orig('Makefile')
    needle = '\t$(Q)$(NEXMON_ROOT)/buildtools/$(B43VERSION)/debug/b43-beautifier'
    pos = s.index(needle)
    end = s.index('\n', pos)
    s = s[:pos] + 'ifeq ($(findstring bcm4366c0,$(FWUCODE)), )\n' + s[pos:end] + '\nendif' + s[end:]
    s = s.replace('>log/patch.log || true', '>log/patch.log')
    s = s.replace('--cpp-args -DRXE_RXHDR_LEN=', '--cpp-args -I$(NEXMON_ROOT)/buildtools/$(B43VERSION)/debug/include -DRXE_RXHDR_LEN=')
    workdir.joinpath('Makefile').write_text(s)


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError(f"stock microcode anchor is not unique: {old!r}")
    return source.replace(old, new, 1)


def microcode(nexmon, workdir):
    firmware = nexmon / "firmwares/bcm4366c0/10_10_122_20"
    disassembler = nexmon / "buildtools/b43-v2/disassembler/b43-dasm"
    with tempfile.TemporaryDirectory() as temporary:
        stock_path = Path(temporary) / "stock.asm"
        snapshot_path = Path(temporary) / "snapshot.asm"
        subprocess.run([str(disassembler), str(firmware / "nonmuucode.bin"),
                        str(stock_path), "--arch", "15", "--format", "raw-le32"], check=True)
        stock = stock_path.read_text()
        patched = replace_once(stock, "%arch 15", (RESOURCES / "ucode_prefix.inc").read_text() + "\n%arch 15")
        patched = replace_once(patched, "L593:\n", "L593:\n xadd CSI_RX_GENERATION, 1, CSI_RX_GENERATION\n")
        words = (0xb80, 0xba0, 0xba3, 0xba4, 0xba7, 0xba8, 0xba9, 0xbaa, 0xbab, 0xbac)
        initialize = "".join(f" mov 0, [0x{address:x}]\n" for address in words)
        patched = replace_once(patched, "L901:\n", "L901:\n" + initialize)
        anchor = "L726:\n\tcalls\tL902\n"
        patched = replace_once(patched, anchor, anchor + (RESOURCES / "ucode_filter.inc").read_text() + "\n")
        anchor = "\t@0\t@0, @0, @0"
        patched = replace_once(patched, anchor, "\n" + (RESOURCES / "ucode_capture.inc").read_text() + "\n" + anchor)
        snapshot_path.write_text(patched)
        result = subprocess.run(["diff", str(stock_path), str(snapshot_path)], text=True, capture_output=True)
        if result.returncode != 1:
            raise RuntimeError("could not generate the microcode patch")
        (workdir / "src" / UCODE_PATCH).write_text(result.stdout)
        return result.stdout


def write_overlay_patch(upstream, workdir, output):
    changes = []
    for filename in ("Makefile", "src/csi_extractor.c", "src/ioctl.c", "src/version.c",
                     "src/" + UCODE_PATCH, "src/ap_snapshot.inc"):
        original = "" if filename == "src/ap_snapshot.inc" else upstream_text(upstream, filename)
        updated = (workdir / filename).read_text()
        changes.extend(difflib.unified_diff(original.splitlines(keepends=True),
                       updated.splitlines(keepends=True),
                       fromfile="/dev/null" if not original else "a/" + filename,
                       tofile="b/" + filename))
    output.write_text("".join(changes))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nexmon", type=Path, required=True)
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--workdir", type=Path, required=True)
    parser.add_argument("--patch", type=Path)
    arguments = parser.parse_args()
    for checkout, revision in ((arguments.nexmon, NEXMON_REVISION), (arguments.upstream, CSI_REVISION)):
        actual = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
        if actual != revision:
            raise ValueError(f"unsupported source revision in {checkout}: {actual}")
    if arguments.workdir.resolve() in (arguments.upstream.resolve(), arguments.nexmon.resolve()):
        raise ValueError("the build directory must be separate from the upstream checkout")
    if not (arguments.workdir / "Makefile").is_file():
        raise ValueError("first extract the pinned CSI git archive into the build directory")
    integrate(arguments.upstream, arguments.workdir)
    generated = microcode(arguments.nexmon, arguments.workdir)
    if generated != (RESOURCES / UCODE_PATCH).read_text():
        raise ValueError("generated microcode differs from the reviewed patch")
    if arguments.patch:
        write_overlay_patch(arguments.upstream, arguments.workdir, arguments.patch)


if __name__ == "__main__":
    main()
