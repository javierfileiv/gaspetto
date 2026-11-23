#!/usr/bin/env python3
"""
Talk to the relay board (NrfLink or GBox) on its serial port.

With no --ota this is a plain console, like tio. With --ota it builds the car
firmware, sends it over the radio through whichever relay is on the port, and
then stays on the console.

The car has no USB command channel, so the image travels over the nRF24 link:
The relay turns a binary stream from its serial port into OTA packets. The stream
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
import shutil
import struct
import subprocess
import sys
import termios
import time
import tty
import zlib

OTA_START = 0x01
# Control codes, not letters: the relay prints hex addresses and log lines
# on the same port, and a plain "F" in "5F" would look like a refusal
OTA_ACK = b"\x06"
OTA_NAK = b"\x15"
OTA_CHUNK = 28
# The relay hands out one credit per group of four packets. The board consumes
# the stream more slowly than the serial link delivers it, so without this the
# port fills up and the UART drops the surplus in silence: the window is what
# keeps the PC from ever getting ahead of what the board has taken
OTA_CREDIT = b"\x11"
OTA_CREDIT_BYTES = 4 * OTA_CHUNK
OTA_WINDOW = OTA_CREDIT_BYTES
# The car erases its staging sector before it can take a byte, and writes the
# record through another erase once the whole image has been checked. The relay
# tries the start three times, so the wait must outlast all of them or we give
# up on a transfer that is still going on
OTA_READY_TIMEOUT = 30.0
OTA_VERDICT_TIMEOUT = 20.0
OTA_LOG_SECONDS = 10.0
# A port that takes nothing for this long is wedged, not merely slow
OTA_STALL_SECONDS = 10.0
LINK_BAUD = termios.B115200
LINK_BOOT_WAIT = 2.0
LINK_PORT_ENV = "GASPETTO_LINK_PORT"
# Kept so existing shells and the DFU upload keep working
LEGACY_PORT_ENV = "GCAR_NRF_PORT"
LINK_PORT_DEFAULT = "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode_337533823134-if00"
GCAR_ENV = "gcar"
GBOX_ENV = "blackpill_f411ce"
# The box jumps to the STM32 ROM bootloader and re-enumerates as this device;
# the application lives at the start of the flash, under the ROM's own layout
DFU_VID = "0483"
DFU_PID = "df11"
DFU_TIMEOUT = 10.0
GBOX_APP_ADDRESS = "0x08000000"


def _root():
    # __file__ only exists when python runs this file, never under SCons
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _gcar_project():
    return os.path.join(_root(), "soft", "pio", "GCar_pio")


def _gbox_project():
    return os.path.join(_root(), "soft", "pio", "GBox_pio")


def _default_firmware():
    return os.path.join(_gcar_project(), ".pio", "build", GCAR_ENV, "firmware.bin")


def _default_gbox_firmware():
    return os.path.join(_gbox_project(), ".pio", "build", GBOX_ENV, "firmware.bin")


def _find_pio():
    for candidate in ("pio", "platformio"):
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            path = os.path.join(directory, candidate)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return path
    return None


def _build_project(project, environment, label, firmware):
    pio = _find_pio()
    if pio is None:
        print("[link] PlatformIO not found: cannot rebuild")
        return 1

    print(f"[link] Rebuilding the {label} firmware ({environment}) ...", flush=True)
    result = subprocess.call([pio, "run", "-d", project, "-e", environment])
    if result != 0:
        print(f"[link] Build failed (exit {result})")
        return result

    if os.path.exists(firmware):
        print(f"[link] Built {os.path.getsize(firmware)} bytes", flush=True)
    return 0


def _build():
    return _build_project(_gcar_project(), GCAR_ENV, "car", _default_firmware())


def _build_gbox():
    return _build_project(_gbox_project(), GBOX_ENV, "box", _default_gbox_firmware())


def _dfu_device_present():
    try:
        out = subprocess.check_output(["dfu-util", "-l"], stderr=subprocess.STDOUT, text=True)
        return f"{DFU_VID}:{DFU_PID}" in out
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def _request_box_dfu(port):
    """The box shell listens for a line, so the command travels as text; the
    command is the same `dfu` an operator types in tio"""
    try:
        fd = _open_port(port, reset=False)
    except (OSError, termios.error) as error:
        print(f"[box] Could not open {port}: {error}")
        return False

    try:
        print(f"[box] Asking {port} to enter DFU ...", flush=True)
        _write_all(fd, b"dfu\n")
        time.sleep(0.3)
        # Show the box's echo, then let it jump: the port disappears on the
        # ROM side and dfu-util takes over
        while select.select([fd], [], [], 0)[0]:
            try:
                data = os.read(fd, 256)
            except BlockingIOError:
                break
            if not data:
                break
            sys.stdout.write(data.decode(errors="ignore"))
            sys.stdout.flush()
    finally:
        os.close(fd)

    return True


def _wait_for_dfu(timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _dfu_device_present():
            return True
        time.sleep(0.5)
    return False


def _flash_box(firmware):
    dfu = shutil.which("dfu-util")
    if dfu is None:
        print("[box] dfu-util not found")
        return 1

    cmd = [
        dfu,
        "-d", f"{DFU_VID}:{DFU_PID}",
        "-a", "0",
        "-s", f"{GBOX_APP_ADDRESS}:leave",
        "-D", firmware,
    ]
    print(" ".join(cmd), flush=True)
    return subprocess.call(cmd)


def _flash_box_mode(port, firmware, build):
    if build:
        code = _build_gbox()
        if code != 0:
            return code

    if not os.path.exists(firmware):
        print(f"[box] No firmware at {firmware}")
        return 1

    print(f"[box] Firmware: {firmware}", flush=True)
    print(f"[box] Port: {port}", flush=True)

    if shutil.which("dfu-util") is None:
        print("[box] dfu-util not found")
        return 1

    if _dfu_device_present():
        print("[box] DFU device already present, skipping the serial command")
    else:
        if not _request_box_dfu(port):
            return 1
        print(f"[box] Waiting up to {DFU_TIMEOUT:g}s for the DFU device ...", flush=True)
        if not _wait_for_dfu(DFU_TIMEOUT):
            print("[box] ERROR: no DFU device appeared. Check the USB cable, or hold BOOT0 "
                  "and press RESET.")
            return 1

    code = _flash_box(firmware)
    if code == 0:
        print("[box] Flash complete.", flush=True)
    return code


def _resolve_port():
    override = os.environ.get(LINK_PORT_ENV) or os.environ.get(LEGACY_PORT_ENV)
    if override:
        return override
    if os.path.exists(LINK_PORT_DEFAULT):
        return LINK_PORT_DEFAULT

    print(f"[link] link port missing: {LINK_PORT_DEFAULT}")
    for candidate in sorted(glob.glob("/dev/serial/by-id/*")):
        print(f"[link]   {candidate}")
    print(f"[link] Set {LINK_PORT_ENV}=<port> (or {LEGACY_PORT_ENV}) to point at the relay board")
    return None


def _open_port(port, reset=True):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNPAR
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = LINK_BAUD
    attrs[5] = LINK_BAUD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    if reset:
        # Pulse DTR so the relay board restarts cleanly, then let its banner
        # come and go before the binary stream starts: a start byte eaten by
        # the bootloader would desynchronise the whole transfer
        try:
            fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR))
            time.sleep(0.05)
            fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
        except OSError:
            pass  # not every serial device exposes DTR
        time.sleep(LINK_BOOT_WAIT)

        # Drop whatever the banner printed
        while select.select([fd], [], [], 0)[0]:
            try:
                if not os.read(fd, 256):
                    break
            except BlockingIOError:
                break

    return fd


def _remote_echoes(port):
    """The tool console does not echo, the box shell does. Typing into the box
    with a local echo as well would double every key."""
    return "ttyACM" not in os.path.realpath(port)


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

        # Anything else is text from the relay: show it as it arrives
        sys.stdout.write(data.decode(errors="ignore"))
        sys.stdout.flush()

    return None


def _show_logs(fd, seconds):
    """Stay on the port after the transfer.

    The car resets, the bootloader copies the image, then the application
    starts and logs its boot lines. Closing here would lose exactly the lines
    that say the update worked, so the port stays open a few more seconds.
    """
    print(f"[link] Listening for the car's logs for {seconds:g} s ...", flush=True)

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


def _stay_attached(fd, echo=True):
    """Hand the console over to the relay once the image is sent.

    The port stays open, so the board is not reset again and every key reaches
    it at once. Opening a second terminal would take the port away from this
    process, and a serial port is not shared: the two readers would split the
    bytes between them.

    The terminal is put in cbreak mode so one key is enough, as on the tool's
    own console. Ctrl-] leaves.
    """
    stdin_fd = sys.stdin.fileno()
    saved = termios.tcgetattr(stdin_fd)
    tty.setcbreak(stdin_fd)
    print("[link] Console attached: type, Ctrl-] to leave.", flush=True)

    try:
        while True:
            ready, _, _ = select.select([fd, stdin_fd], [], [], 0.2)

            if fd in ready:
                try:
                    data = os.read(fd, 256)
                except BlockingIOError:
                    data = None
                if data is not None:
                    if not data:
                        print("\n[link] The port closed.", flush=True)
                        return
                    sys.stdout.write(data.decode(errors="ignore"))
                    sys.stdout.flush()

            if stdin_fd in ready:
                try:
                    typed = os.read(stdin_fd, 256)
                except BlockingIOError:
                    continue
                if not typed:
                    return

                leaving = b"\x1d" in typed
                if leaving:
                    typed = typed.split(b"\x1d", 1)[0]

                if typed:
                    # The tool does not echo, and the terminal no longer does:
                    # show the keys so a silent command is still visible. The
                    # box shell echoes on its own, so its echo is left off
                    if echo:
                        sys.stdout.write("".join(ch for ch in typed.decode(errors="ignore")
                                                 if ch.isprintable()))
                        sys.stdout.flush()
                    _write_all(fd, typed)

                if leaving:
                    return
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, saved)


def _image_payload(image):
    if len(image) > 16 and image[-8:-5] == b"UFD" and image[-5] == 16:
        return image[:-16]
    return image


def _write_image(fd, payload):
    """Dump the image, never more than a window ahead of the relay.

    The relay board reads at its own pace, slower than the line delivers, and
    its serial buffer is small. The relay therefore sends one credit per group of
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

            # Anything else is text from the relay: show it as it arrives. This
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
                print("[link] The relay stopped taking bytes")
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
            print(f"[link] {percent}% ({sent / elapsed / 1024.0:.1f} kB/s)", flush=True)
            mark = percent - percent % 10 + 10

    elapsed = time.time() - started
    print(f"[link] image dumped in {elapsed:.1f} s ({total / elapsed / 1024.0:.1f} kB/s)",
          flush=True)
    return True


def _stream(fd, payload):
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    chunks = (len(payload) + OTA_CHUNK - 1) // OTA_CHUNK

    print(f"[link] {len(payload)} bytes, CRC32 0x{crc:08X}, {chunks} packets", flush=True)

    _write_all(fd, bytes([OTA_START]) + struct.pack("<II", len(payload), crc))

    # The car erases its staging sector first, which takes a couple of seconds,
    # and only then is it able to receive anything
    ready = _wait_ack(fd, OTA_READY_TIMEOUT)
    if ready is None:
        print("[link] The relay never started: check the radio")
        return 1
    if not ready:
        print("[link] The car refused the transfer")
        return 1

    started = time.time()
    if not _write_image(fd, payload):
        print("[link] The car stopped the transfer")
        return 1

    # Only now does the car check the image against the CRC and write the
    # record, so the verdict arrives well after the last byte
    final = _wait_ack(fd, OTA_VERDICT_TIMEOUT)
    if final is None:
        print("[link] The car did not confirm the image")
        return 1
    if not final:
        print("[link] The car refused the image")
        return 1

    elapsed = time.time() - started
    print(f"[link] {chunks} packets in {elapsed:.1f} s", flush=True)
    return 0


def _usage():
    print("usage: gaspetto_link.py [--ota | --box] [--no-build] [--port <port>] [--logs N]")
    print("                        [--no-stay] [--echo|--no-echo] [firmware.bin]")
    print("       no option: attach to the port like tio, Ctrl-] to leave")
    print("       --ota: rebuild the car firmware, send it, then stay on the console")
    print("       --box (--dfu): rebuild the box firmware, ask its serial 'dfu' command,")
    print("                      then flash it with dfu-util")
    print("       --no-build: with --ota or --box, use the binary already in the build")
    print("       --logs N: read the port for N seconds and leave, no hand-off")
    print("       --no-stay: with --ota, use the log window instead of the console")
    print("       --echo / --no-echo: force or suppress the local echo;")
    print("                 a ttyACM target (the box) defaults to no local echo")
    print(f"       default car firmware: {_default_firmware()}")
    print(f"       default box firmware: {_default_gbox_firmware()}")
    print(f"       default port: {LINK_PORT_DEFAULT}")


def _main(argv):
    if "-h" in argv or "--help" in argv:
        _usage()
        return 0

    ota = "--ota" in argv
    box = "--box" in argv or "--dfu" in argv
    if ota and box:
        print("[link] --ota flashes the car, --box flashes the box: pick one")
        return 1

    port = None
    if "--port" in argv:
        index = argv.index("--port")
        if index + 1 < len(argv):
            port = argv[index + 1]

    no_stay = "--no-stay" in argv
    log_value = None
    log_seconds = OTA_LOG_SECONDS
    if "--logs" in argv:
        index = argv.index("--logs")
        if index + 1 < len(argv):
            log_value = argv[index + 1]
            log_seconds = float(log_value)

    skip = {port, log_value}
    explicit = [arg for arg in argv if not arg.startswith("-") and arg not in skip]

    echo = None
    if "--echo" in argv:
        echo = True
    if "--no-echo" in argv:
        echo = False

    if box:
        firmware = _default_gbox_firmware()
        build = "--no-build" not in argv

        if explicit:
            firmware = explicit[0]
            build = False

        # A box already in DFU has no CDC port left: skip the search so the
        # tool can still complete the flash
        if not _dfu_device_present():
            port = port or _resolve_port()
            if port is None:
                return 1
        else:
            port = port or "(already in DFU)"

        return _flash_box_mode(port, firmware, build)

    port = port or _resolve_port()
    if port is None:
        return 1

    if echo is None:
        echo = _remote_echoes(port)

    if not ota:
        # Plain console: the port is opened without a reset, so the board's
        # current output is not thrown away, and nothing is built or sent
        try:
            fd = _open_port(port, reset=False)
        except OSError as error:
            print(f"[link] Could not open {port}: {error}")
            return 1

        print(f"[link] {port} open, Ctrl-] to leave", flush=True)
        try:
            if "--logs" in argv:
                _show_logs(fd, log_seconds)
            elif sys.stdin.isatty() and sys.stdout.isatty():
                _stay_attached(fd, echo)
            else:
                _show_logs(fd, log_seconds)
        except KeyboardInterrupt:
            print()
        finally:
            os.close(fd)
        return 0

    firmware = _default_firmware()

    if explicit:
        firmware = explicit[0]
    elif "--no-build" not in argv:
        code = _build()
        if code != 0:
            return code

    if not os.path.exists(firmware):
        print(f"[link] No firmware at {firmware}")
        return 1

    with open(firmware, "rb") as handle:
        payload = _image_payload(handle.read())

    print(f"[link] Firmware: {firmware}", flush=True)
    print(f"[link] Port: {port}", flush=True)

    try:
        fd = _open_port(port, reset=True)
    except OSError as error:
        print(f"[link] Could not open {port}: {error}")
        return 1

    # A terminal can take the tool over directly, which is what an operator
    # wants after a flash: no second program, and no port to fight over. An
    # explicit --logs asks for the timed window instead
    stay = (not no_stay and "--logs" not in argv and sys.stdin.isatty() and
            sys.stdout.isatty())

    try:
        code = _stream(fd, payload)
        if code == 0 and stay:
            _stay_attached(fd, echo)
        elif log_seconds > 0:
            _show_logs(fd, log_seconds)
    except KeyboardInterrupt:
        print()
        code = 0
    finally:
        os.close(fd)

    if code == 0:
        print("[link] Image sent.")
    return code


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
