"""Deploy Pi 1's phone-facing Mosquitto listener and eduroam bridge."""

import os
import posixpath
import argparse
from datetime import datetime, timezone
from pathlib import Path

import paramiko


ROOT = Path(__file__).resolve().parent
LOCAL_CONFIG = ROOT / "mosquitto-pi1-relay.conf"
PI_USER = "wiloc"
REMOTE_CONFIG = "/etc/mosquitto/conf.d/wiros-relay.conf"
REMOTE_OVERRIDE = "/etc/systemd/system/mosquitto.service.d/wiros.conf"
REMOTE_CSI_CONFIG = "/opt/wiros_csi_node/src/pythonNoRos/config.json"
PASSWORD = os.environ["WIRES_PI_PASSWORD"]


def run(client, command, sudo=False, check=True):
    stdin, stdout, stderr = client.exec_command(command, timeout=30)
    if sudo:
        stdin.write(PASSWORD + "\n")
        stdin.flush()
    output = stdout.read().decode(errors="replace")
    error = stderr.read().decode(errors="replace")
    status = stdout.channel.recv_exit_status()
    if check and status:
        raise RuntimeError(f"command failed ({status}): {command}\n{output}{error}")
    return status, output.strip(), error.strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.48.20")
    parser.add_argument("--port", type=int, default=22)
    args = parser.parse_args()
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, port=args.port, username=PI_USER, password=PASSWORD, timeout=10)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    temporary = f"/home/{PI_USER}/wiros-relay.conf.new-{stamp}"
    backup = f"{REMOTE_CONFIG}.bak-{stamp}"
    existed = False
    try:
        sftp = client.open_sftp()
        sftp.put(str(LOCAL_CONFIG), temporary)
        sftp.close()
        existed = run(client, f"test -f {REMOTE_CONFIG}", check=False)[0] == 0
        if existed:
            run(client, f"sudo -S cp {REMOTE_CONFIG} {backup}", sudo=True)
        run(
            client,
            f"sudo -S install -o root -g root -m 0644 {temporary} {REMOTE_CONFIG}",
            sudo=True,
        )
        run(
            client,
            f"sudo -S sh -c 'test ! -f {REMOTE_OVERRIDE} || mv {REMOTE_OVERRIDE} {REMOTE_OVERRIDE}.disabled'",
            sudo=True,
        )
        run(client, "sudo -S systemctl daemon-reload", sudo=True)
        run(client, "sudo -S systemctl enable mosquitto", sudo=True)
        status, output, error = run(
            client, "sudo -S systemctl restart mosquitto", sudo=True, check=False
        )
        if status:
            if existed:
                run(client, f"sudo -S cp {backup} {REMOTE_CONFIG}", sudo=True)
            else:
                run(client, f"sudo -S rm -f {REMOTE_CONFIG}", sudo=True)
            run(client, "sudo -S systemctl restart mosquitto", sudo=True)
            raise RuntimeError(f"Mosquitto restart failed; restored prior config\n{output}{error}")
        # The local CSI collector and the bridge must target the same central broker.
        run(
            client,
            f"sudo -S sed -i 's/192\\.168\\.50\\.20/192.168.50.1/g' {REMOTE_CSI_CONFIG}",
            sudo=True,
        )
        run(client, "sudo -S systemctl enable wiros-csi-rpi1.service", sudo=True)
        run(client, "sudo -S systemctl restart wiros-csi-rpi1.service", sudo=True)
        state = run(client, "systemctl is-active mosquitto")[1]
        listener = run(client, "ss -ltn | grep '192.168.48.20:1883'")[1]
        print(f"mosquitto={state}")
        print(listener)
        print(f"router1_csi={run(client, 'systemctl is-active wiros-csi-rpi1.service', check=False)[1]}")
        print(f"backup={backup if existed else 'not needed (new file)'}")
    finally:
        run(client, f"rm -f {temporary}", check=False)
        client.close()


if __name__ == "__main__":
    main()
