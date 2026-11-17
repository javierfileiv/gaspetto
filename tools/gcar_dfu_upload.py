#!/usr/bin/env python3
"""
Flash the GCar through its DFU bootloader, asking the car over the radio.

Two ways in:

    pio run -e gcar -t upload
        PlatformIO loads this file as an extra_script, builds, then calls
        dfu_upload() with the binary it just produced.

    python tools/gcar_dfu_upload.py [--no-build | --build-only] [firmware.bin]
        Standalone: rebuild the car firmware first, then flash it. The
        rebuild is the default so the flashed image always matches the
        sources; --no-build skips it and flashes what is already there, and
        an explicit firmware path is taken as-is without rebuilding.

Why the radio: the car has no USB CDC command channel, so the box trick
(send "dfu" on the serial port) cannot work. NrfLink listens on its own
serial port and transmits a tagged DFU_REQUEST event when it receives 'f',
which is why this script drives NrfLink and not the car.

Flow:
  1. If a DFU device is already enumerated, skip the radio step.
  2. Open NrfLink's port, pulse DTR (an Uno resets on it), wait for its boot,
     then write 'f'.
  3. Wait up to DFU_TIMEOUT for 0483:df11, then flash with dfu-util.
"""

import fcntl
import glob
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import termios
import time
import zlib

DFU_VID = "0483"
DFU_PID = "df11"
DFU_TIMEOUT = 10  # seconds to wait for the car to re-enumerate
NRF_BAUD = termios.B115200
NRF_BOOT_WAIT = 2.0  # Optiboot holds the Uno ~1s before the app runs
NRF_PORT_ENV = "GCAR_NRF_PORT"
NRF_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
GCAR_ENV = "gcar"
# The application starts after the bootloader's 32 kB; the bootloader itself is
# flashed once at 0x08000000 and is never touched by an application upload
GCAR_APP_ADDRESS = "0x08008000"
# Validation record the bootloader reads before jumping: magic, format,
# image length and CRC32. It lives at the start of the free sector 5, which
# writing it erases without touching the application
GCAR_META_ADDRESS = "0x08020000"


def _root():
    # __file__ is only defined when python runs this file directly: SCons
    # executes it with its own globals, so this must not run at import time
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _gcar_project():
    return os.path.join(_root(), "soft", "pio", "GCar_pio")


def _default_firmware():
    return os.path.join(_gcar_project(), ".pio", "build", GCAR_ENV, "firmware.bin")


def _dfu_device_present():
    try:
        out = subprocess.check_output(
            ["dfu-util", "-l"], stderr=subprocess.STDOUT, text=True
        )
        return (DFU_VID + ":" + DFU_PID) in out
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def _build():
    pio = shutil.which("pio")
    if pio is None:
        print("[gcar_dfu] PlatformIO (pio) not found: cannot rebuild")
        return 1

    print(f"[gcar_dfu] Rebuilding the car firmware ({GCAR_ENV}) ...", flush=True)
    result = subprocess.call([pio, "run", "-d", _gcar_project(), "-e", GCAR_ENV])
    if result != 0:
        print(f"[gcar_dfu] Build failed (exit {result}), nothing flashed")
        return result

    firmware = _default_firmware()
    if os.path.exists(firmware):
        size = os.path.getsize(firmware)
        stamp = time.strftime("%H:%M:%S", time.localtime(os.path.getmtime(firmware)))
        print(f"[gcar_dfu] Built {size} bytes at {stamp}", flush=True)
    return 0


def _resolve_nrf_port():
    override = os.environ.get(NRF_PORT_ENV)
    if override:
        return override
    if os.path.exists(NRF_PORT_DEFAULT):
        return NRF_PORT_DEFAULT

    print(f"[gcar_dfu] NrfLink port missing: {NRF_PORT_DEFAULT}")
    candidates = sorted(glob.glob("/dev/serial/by-id/*"))
    if candidates:
        print("[gcar_dfu] Ports currently present:")
        for candidate in candidates:
            print(f"[gcar_dfu]   {candidate}")
    print(f"[gcar_dfu] Set {NRF_PORT_ENV}=<port> to point at the Uno")
    return None


def _send_dfu_request(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = termios.IGNPAR
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = NRF_BAUD
        attrs[5] = NRF_BAUD
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        # Pulse DTR so the Uno restarts cleanly, then let Optiboot time out
        # and NrfLink print its banner before the key is written
        fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR))
        time.sleep(0.05)
        fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
        time.sleep(NRF_BOOT_WAIT)

        print(f"[gcar_dfu] Asking NrfLink on {port} to send the DFU request ...")
        os.write(fd, b"f")
        time.sleep(0.2)
    finally:
        os.close(fd)


def _wait_for_dfu():
    deadline = time.time() + DFU_TIMEOUT
    while time.time() < deadline:
        if _dfu_device_present():
            return True
        time.sleep(0.5)
    return False


def _enter_dfu():
    port = _resolve_nrf_port()
    if port is None:
        return False

    try:
        _send_dfu_request(port)
    except OSError as error:
        print(f"[gcar_dfu] Could not drive {port}: {error}")
        return False

    print(f"[gcar_dfu] Waiting up to {DFU_TIMEOUT}s for the car to enumerate ...")
    if _wait_for_dfu():
        return True

    print(
        "[gcar_dfu] ERROR: no DFU device. Check that the car is powered and in "
        "range, that NrfLink runs the firmware with the 'f' key, then recover "
        "with RESET, or BOOT0 plus RESET."
    )
    return False


def _dfu_write(address, firmware, leave):
    cmd = [
        "dfu-util",
        "-d", f"{DFU_VID}:{DFU_PID}",
        "-a", "0",
        "-s", f"{address}:leave" if leave else address,
        "-D", firmware,
    ]
    print(" ".join(cmd), flush=True)
    return subprocess.call(cmd)


def _write_metadata(image, crc):
    """Record the length and CRC the bootloader will verify before jumping"""
    payload = struct.pack("<4sHHII", b"GCAR", 1, 0, len(image), crc)
    handle, path = tempfile.mkstemp(prefix="gcar_meta_", suffix=".bin")

    try:
        os.write(handle, payload)
    finally:
        os.close(handle)

    try:
        return _dfu_write(GCAR_META_ADDRESS, path, leave=True)
    finally:
        os.unlink(path)


def _image_payload(image):
    """The 16-byte DFU suffix PIO appends is not part of the image"""
    if len(image) > 16 and image[-8:-5] == b"UFD" and image[-5] == 16:
        return image[:-16]
    return image


def _dfu_detach():
    return subprocess.call(["dfu-util", "-d", f"{DFU_VID}:{DFU_PID}", "-e"])


def _flash(firmware, bad_crc=False, no_record=False):
    with open(firmware, "rb") as handle:
        image = handle.read()

    payload = _image_payload(image)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    print(
        f"[gcar_dfu] {len(payload)} bytes, CRC32 0x{crc:08X}",
        flush=True,
    )

    print("[gcar_dfu] Uploading the application ...", flush=True)
    result = _dfu_write(GCAR_APP_ADDRESS, firmware, leave=False)
    if result != 0:
        print(f"[gcar_dfu] dfu-util failed on the application (exit {result})")
        return result

    if no_record:
        print("[gcar_dfu] Leaving the record untouched, as asked", flush=True)
        return _dfu_detach()

    if bad_crc:
        crc ^= 0xFFFFFFFF

    print("[gcar_dfu] Uploading the validation record ...", flush=True)
    result = _write_metadata(payload, crc)
    if result != 0:
        print(f"[gcar_dfu] dfu-util failed on the record (exit {result})")
        print("[gcar_dfu] The bootloader will refuse this image and blink")
        return result

    print("[gcar_dfu] Upload complete.")
    return 0


def _upload(firmware, bad_crc=False, no_record=False):
    print(f"[gcar_dfu] Firmware: {firmware}", flush=True)

    if _dfu_device_present():
        print("[gcar_dfu] DFU device already present - skipping the radio step")
    elif not _enter_dfu():
        return 1

    return _flash(firmware, bad_crc=bad_crc, no_record=no_record)


def _usage():
    print("usage: gcar_dfu_upload.py [--no-build | --build-only] [firmware.bin]")
    print("       no option: rebuild the car firmware, then flash it")
    print("       --no-build: flash the binary already in the build directory")
    print("       --build-only: rebuild and stop there")
    print("       --bad-crc: write a wrong CRC on purpose, to see the bootloader")
    print("                  refuse the image with three LED blinks")
    print("       --no-record: flash the application and leave the record alone")
    print(f"       default firmware: {_default_firmware()}")
    print("       or let PlatformIO drive it: pio run -e gcar -t upload")


def _main(argv):
    if "-h" in argv or "--help" in argv:
        _usage()
        return 0

    explicit = [arg for arg in argv if not arg.startswith("-")]

    if "--build-only" in argv:
        return _build()

    firmware = _default_firmware()
    if explicit:
        # A named binary is taken as it is: rebuilding would replace it
        firmware = explicit[0]
    elif "--no-build" not in argv:
        code = _build()
        if code != 0:
            return code

    if not os.path.exists(firmware):
        print(f"[gcar_dfu] No firmware at {firmware}")
        print("[gcar_dfu] Build it first with: pio run -e gcar")
        return 1

    return _upload(firmware, bad_crc="--bad-crc" in argv, no_record="--no-record" in argv)


def dfu_upload(source, target, env):
    code = _upload(str(source[0]))
    if code != 0:
        sys.exit(code)


try:
    Import("env")  # noqa: F821 - defined only when PlatformIO loads this file
except NameError:
    # Plain python invocation
    sys.exit(_main(sys.argv[1:]))
else:
    env.Replace(UPLOADCMD=dfu_upload)  # noqa: F821
