"""
PlatformIO extra_script: auto-enter DFU via USB CDC, then upload with dfu-util.

Flow:
  1. If device is already in DFU mode (0483:df11), skip CDC step.
  2. Otherwise, find the CDC serial port, send "dfu\n", wait for re-enumeration.
  3. Run dfu-util to flash the firmware.
"""

Import("env")  # noqa: F821 – PlatformIO SCons env

import subprocess
import sys
import time
import glob

DFU_VID = "0483"
DFU_PID = "df11"
DFU_TIMEOUT = 10  # seconds to wait for DFU re-enumeration
CDC_BAUD = 115200


def _find_cdc_ports():
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    return sorted(candidates)


def _dfu_device_present():
    try:
        out = subprocess.check_output(
            ["dfu-util", "-l"], stderr=subprocess.STDOUT, text=True
        )
        return (DFU_VID + ":" + DFU_PID) in out
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def _trigger_dfu_via_cdc():
    try:
        import serial  # pyserial
    except ImportError:
        print("[dfu_upload] pyserial not found – skipping CDC trigger")
        return False

    ports = _find_cdc_ports()
    if not ports:
        print("[dfu_upload] No CDC port found – skipping CDC trigger")
        return False

    for port in ports:
        try:
            print(f"[dfu_upload] Sending DFU command to {port} ...")
            with serial.Serial(port, CDC_BAUD, timeout=1) as ser:
                ser.write(b"dfu\n")
                time.sleep(0.1)
            return True
        except Exception as e:
            print(f"[dfu_upload] Could not open {port}: {e}")

    return False


def _wait_for_dfu():
    deadline = time.time() + DFU_TIMEOUT
    while time.time() < deadline:
        if _dfu_device_present():
            return True
        time.sleep(0.5)
    return False


def dfu_upload(source, target, env):
    firmware = str(source[0])
    print(f"[dfu_upload] Firmware: {firmware}")

    if _dfu_device_present():
        print("[dfu_upload] DFU device already present – skipping CDC trigger")
    else:
        _trigger_dfu_via_cdc()
        print(f"[dfu_upload] Waiting up to {DFU_TIMEOUT}s for DFU enumeration ...")
        if not _wait_for_dfu():
            print(
                "[dfu_upload] ERROR: DFU device did not appear. "
                "Hold BOOT0 and press RESET, or check USB connection."
            )
            sys.exit(1)

    print("[dfu_upload] Uploading via dfu-util ...")
    cmd = [
        "dfu-util",
        "-d", f"{DFU_VID}:{DFU_PID}",
        "-a", "0",
        "-s", "0x08000000:leave",
        "-D", firmware,
    ]
    print(" ".join(cmd))
    result = subprocess.call(cmd)
    if result != 0:
        print(f"[dfu_upload] dfu-util failed (exit {result})")
        sys.exit(result)
    print("[dfu_upload] Upload complete.")


env.Replace(UPLOADCMD=dfu_upload)  # noqa: F821
