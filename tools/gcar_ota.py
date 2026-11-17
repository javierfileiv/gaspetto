#!/usr/bin/env python3
"""
Send a firmware image to the GCar over the radio, through NrfLink.

The car has no USB command channel, so the image travels over the nRF24 link:
NrfLink turns a binary stream from its serial port into OTA packets. The stream
is a 0x01 start byte, the image length and its CRC32 as two little-endian
words, then the image itself, 28 bytes at a time. NrfLink answers one status
byte per chunk, so the PC never outruns the radio.

The image is the raw firmware payload: the DFU suffix PlatformIO appends is not
part of it, and the CRC covers only what the receiver will count.
"""

import fcntl
import glob
import os
import select
import struct
import subprocess
import sys
import termios
import time
import zlib

OTA_START = 0x01
# Control codes, not letters: NrfLink prints hex addresses and log lines
# on the same port, and a plain "F" in "5F" would look like a refusal
OTA_ACK = b"\x06"
OTA_NAK = b"\x15"
OTA_CHUNK = 28
OTA_ACK_TIMEOUT = 3.0
OTA_LOG_SECONDS = 10.0
NRF_BAUD = termios.B115200
NRF_BOOT_WAIT = 2.0
NRF_PORT_ENV = "GCAR_NRF_PORT"
NRF_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
GCAR_ENV = "gcar"


def _root():
    # __file__ only exists when python runs this file, never under SCons
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _gcar_project():
    return os.path.join(_root(), "soft", "pio", "GCar_pio")


def _default_firmware():
    return os.path.join(_gcar_project(), ".pio", "build", GCAR_ENV, "firmware.bin")


def _build():
    pio = None
    for candidate in ("pio", "platformio"):
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            path = os.path.join(directory, candidate)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                pio = path
                break
        if pio:
            break

    if pio is None:
        print("[gcar_ota] PlatformIO not found: cannot rebuild")
        return 1

    print(f"[gcar_ota] Rebuilding the car firmware ({GCAR_ENV}) ...", flush=True)
    result = subprocess.call([pio, "run", "-d", _gcar_project(), "-e", GCAR_ENV])
    if result != 0:
        print(f"[gcar_ota] Build failed (exit {result})")
        return result

    firmware = _default_firmware()
    if os.path.exists(firmware):
        print(f"[gcar_ota] Built {os.path.getsize(firmware)} bytes", flush=True)
    return 0


def _resolve_port():
    override = os.environ.get(NRF_PORT_ENV)
    if override:
        return override
    if os.path.exists(NRF_PORT_DEFAULT):
        return NRF_PORT_DEFAULT

    print(f"[gcar_ota] NrfLink port missing: {NRF_PORT_DEFAULT}")
    for candidate in sorted(glob.glob("/dev/serial/by-id/*")):
        print(f"[gcar_ota]   {candidate}")
    print(f"[gcar_ota] Set {NRF_PORT_ENV}=<port> to point at the Uno")
    return None


def _open_port(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNPAR
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = NRF_BAUD
    attrs[5] = NRF_BAUD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # Pulse DTR so the Uno restarts cleanly, then let its banner come and go
    # before the binary stream starts: a start byte eaten by Optiboot would
    # desynchronise the whole transfer
    try:
        fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR))
        time.sleep(0.05)
        fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
    except OSError:
        pass  # not every serial device exposes DTR
    time.sleep(NRF_BOOT_WAIT)

    # Drop whatever the banner printed
    while select.select([fd], [], [], 0)[0]:
        try:
            if not os.read(fd, 256):
                break
        except BlockingIOError:
            break

    return fd


def _write_all(fd, data):
    view = memoryview(data)
    while view:
        try:
            written = os.write(fd, view)
        except BlockingIOError:
            select.select([], [fd], [], 0.1)
            continue
        view = view[written:]


def _wait_ack(fd):
    deadline = time.time() + OTA_ACK_TIMEOUT

    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], max(0.0, deadline - time.time()))
        if not ready:
            break

        # select on a non-blocking tty can report readable with nothing to
        # read, so EAGAIN is not an error here
        try:
            data = os.read(fd, 1)
        except BlockingIOError:
            continue

        if not data:
            break
        if data == OTA_ACK:
            return True
        if data == OTA_NAK:
            return False

        # Anything else is text from NrfLink: show it as it arrives
        sys.stdout.write(data.decode(errors="ignore"))
        sys.stdout.flush()

    return None


def _show_logs(fd, seconds):
    """Stay on the port after the transfer.

    The car resets, the bootloader copies the image, then the application
    starts and logs its boot lines. Closing here would lose exactly the lines
    that say the update worked, so the port stays open a few more seconds.
    """
    print(f"[gcar_ota] Listening for the car's logs for {seconds:g} s ...", flush=True)

    deadline = time.time() + seconds
    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.2)
        if not ready:
            continue

        try:
            data = os.read(fd, 256)
        except BlockingIOError:
            continue
        if not data:
            break

        sys.stdout.write(data.decode(errors="ignore"))
        sys.stdout.flush()


def _image_payload(image):
    if len(image) > 16 and image[-8:-5] == b"UFD" and image[-5] == 16:
        return image[:-16]
    return image


def _stream(fd, payload):
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    chunks = (len(payload) + OTA_CHUNK - 1) // OTA_CHUNK
    sent = 0

    print(f"[gcar_ota] {len(payload)} bytes, CRC32 0x{crc:08X}, {chunks} packets", flush=True)

    _write_all(fd, bytes([OTA_START]) + struct.pack("<II", len(payload), crc))

    started = time.time()
    for index in range(chunks):
        _write_all(fd, payload[index * OTA_CHUNK:(index + 1) * OTA_CHUNK])

        ack = _wait_ack(fd)
        if ack is None:
            print(f"[gcar_ota] No answer after packet {index}: NrfLink stopped")
            return 1, sent
        if not ack:
            print(f"[gcar_ota] Packet {index} was not transmitted (radio error)")
            return 1, sent

        sent += 1
        if chunks >= 20 and index % (chunks // 20) == 0:
            print(f"[gcar_ota] {100 * index // chunks}%", flush=True)

    elapsed = time.time() - started
    print(f"[gcar_ota] {sent} packets in {elapsed:.1f} s", flush=True)

    # The last chunk only proves it was written; the car verifies the whole
    # image and answers after the end packet
    final = _wait_ack(fd)
    if final is None:
        print("[gcar_ota] The car did not confirm the image")
        return 1, sent
    if not final:
        print("[gcar_ota] The car refused the image")
        return 1, sent

    return 0, sent


def _usage():
    print("usage: gcar_ota.py [--no-build] [--port <port>] [firmware.bin]")
    print("       no option: rebuild the car firmware, then send it over the radio")
    print("       --no-build: send the binary already in the build directory")
    print("       --logs N: keep listening to the car for N seconds after the end")
    print(f"                 (default {OTA_LOG_SECONDS:.0f})")
    print(f"       default firmware: {_default_firmware()}")
    print(f"       default port: {NRF_PORT_DEFAULT}")


def _main(argv):
    if "-h" in argv or "--help" in argv:
        _usage()
        return 0

    port = None
    if "--port" in argv:
        index = argv.index("--port")
        if index + 1 < len(argv):
            port = argv[index + 1]

    log_value = None
    log_seconds = OTA_LOG_SECONDS
    if "--logs" in argv:
        index = argv.index("--logs")
        if index + 1 < len(argv):
            log_value = argv[index + 1]
            log_seconds = float(log_value)

    skip = {port, log_value}
    explicit = [arg for arg in argv if not arg.startswith("-") and arg not in skip]
    firmware = _default_firmware()

    if explicit:
        firmware = explicit[0]
    elif "--no-build" not in argv:
        code = _build()
        if code != 0:
            return code

    if not os.path.exists(firmware):
        print(f"[gcar_ota] No firmware at {firmware}")
        return 1

    port = port or _resolve_port()
    if port is None:
        return 1

    with open(firmware, "rb") as handle:
        payload = _image_payload(handle.read())

    print(f"[gcar_ota] Firmware: {firmware}", flush=True)
    print(f"[gcar_ota] Port: {port}", flush=True)

    try:
        fd = _open_port(port)
    except OSError as error:
        print(f"[gcar_ota] Could not open {port}: {error}")
        return 1

    try:
        code, _ = _stream(fd, payload)
        if log_seconds > 0:
            _show_logs(fd, log_seconds)
    finally:
        os.close(fd)

    if code == 0:
        print("[gcar_ota] Image sent.")
    return code


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
