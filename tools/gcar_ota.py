#!/usr/bin/env python3
"""
Send a firmware image to the GCar over the radio, through NrfLink.

The car has no USB command channel, so the image travels over the nRF24 link:
NrfLink turns a binary stream from its serial port into OTA packets. The stream
is a 0x01 start byte, the image length and its CRC32 as two little-endian
words, then the image itself, 28 bytes at a time. Nothing comes back per
packet: the nRF24 acknowledges each one in hardware, and the serial port's own
flow control paces the dump. The car only answers when it is ready to take the
image, and once more when the CRC verdict is in.

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
# The relay hands out one credit per group of four packets. The Uno consumes
# the stream more slowly than the serial link delivers it, so without this the
# port fills up and the UART drops the surplus in silence: the window is what
# keeps the PC from ever getting ahead of what the board has taken
OTA_CREDIT = b"\x11"
OTA_CREDIT_BYTES = 4 * OTA_CHUNK
OTA_WINDOW = OTA_CREDIT_BYTES
# The car erases its staging sector before it can take a byte, and writes the
# record through another erase once the whole image has been checked. NrfLink
# tries the start three times, so the wait must outlast all of them or we give
# up on a transfer that is still going on
OTA_READY_TIMEOUT = 30.0
OTA_VERDICT_TIMEOUT = 20.0
OTA_LOG_SECONDS = 10.0
# A port that takes nothing for this long is wedged, not merely slow
OTA_STALL_SECONDS = 10.0
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


def _wait_ack(fd, timeout):
    deadline = time.time() + timeout

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
        if data == OTA_CREDIT:
            continue  # flow control left over from the dump

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


def _write_image(fd, payload):
    """Dump the image, never more than a window ahead of the relay.

    The Uno reads at its own pace, slower than the line delivers, and its
    serial buffer is small. The relay therefore sends one credit per group of
    packets it has taken, and the count of credits is what allows the PC to
    keep writing: the stream stays full without ever overrunning the board.
    """
    total = len(payload)
    sent = 0
    granted = 0
    mark = 10
    started = time.time()
    last_change = started
    previous = 0

    while sent < total:
        ready, _, _ = select.select([fd], [], [], 0)
        if ready:
            try:
                data = os.read(fd, 256)
            except BlockingIOError:
                data = b""
            granted += data.count(OTA_CREDIT) * OTA_CREDIT_BYTES

            # Anything else is text from NrfLink: show it as it arrives. This
            # runs before the refusal check, because the relay writes its
            # reason just before the refusal byte and both come in one read
            text = data.replace(OTA_CREDIT, b"").replace(OTA_ACK, b"").replace(OTA_NAK, b"")
            if text:
                sys.stdout.write(text.decode(errors="ignore"))
                sys.stdout.flush()

            if OTA_NAK in data:
                return False

        budget = granted + OTA_WINDOW - sent
        if budget <= 0:
            select.select([fd], [], [], 0.002)
            if time.time() - last_change > OTA_STALL_SECONDS:
                print("[gcar_ota] The relay stopped taking bytes")
                return False
            continue

        _, writable, _ = select.select([], [fd], [], 0.05)
        if not writable:
            continue

        try:
            written = os.write(fd, payload[sent:sent + min(budget, 1024)])
        except BlockingIOError:
            continue

        sent += written
        if sent != previous:
            previous = sent
            last_change = time.time()

        percent = 100 * sent // total
        if percent >= mark:
            elapsed = max(1e-3, time.time() - started)
            print(f"[gcar_ota] {percent}% ({sent / elapsed / 1024.0:.1f} kB/s)", flush=True)
            mark = percent - percent % 10 + 10

    elapsed = time.time() - started
    print(f"[gcar_ota] image dumped in {elapsed:.1f} s ({total / elapsed / 1024.0:.1f} kB/s)",
          flush=True)
    return True


def _stream(fd, payload):
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    chunks = (len(payload) + OTA_CHUNK - 1) // OTA_CHUNK

    print(f"[gcar_ota] {len(payload)} bytes, CRC32 0x{crc:08X}, {chunks} packets", flush=True)

    _write_all(fd, bytes([OTA_START]) + struct.pack("<II", len(payload), crc))

    # The car erases its staging sector first, which takes a couple of seconds,
    # and only then is it able to receive anything
    ready = _wait_ack(fd, OTA_READY_TIMEOUT)
    if ready is None:
        print("[gcar_ota] NrfLink never started: check the radio")
        return 1
    if not ready:
        print("[gcar_ota] The car refused the transfer")
        return 1

    started = time.time()
    if not _write_image(fd, payload):
        print("[gcar_ota] The car stopped the transfer")
        return 1

    # Only now does the car check the image against the CRC and write the
    # record, so the verdict arrives well after the last byte
    final = _wait_ack(fd, OTA_VERDICT_TIMEOUT)
    if final is None:
        print("[gcar_ota] The car did not confirm the image")
        return 1
    if not final:
        print("[gcar_ota] The car refused the image")
        return 1

    elapsed = time.time() - started
    print(f"[gcar_ota] {chunks} packets in {elapsed:.1f} s", flush=True)
    return 0


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
        code = _stream(fd, payload)
        if log_seconds > 0:
            _show_logs(fd, log_seconds)
    finally:
        os.close(fd)

    if code == 0:
        print("[gcar_ota] Image sent.")
    return code


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
