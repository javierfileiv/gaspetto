#!/usr/bin/env python3
"""Realtime plot for orientation DATA lines from serial.

Usage:
  python3 plot_orientation.py --port /dev/ttyUSB0 --baud 115200

DATA protocol (CSV after initial header line):
    DATA,time_ms,yaw_deg,pitch_deg,roll_deg,yaw_error_deg,mode,left,right,cmd

Dependencies: pip install pyserial matplotlib
"""
import argparse
import sys
import collections
import signal
import csv
import os
from typing import Dict, Deque, Any
import serial
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Set default for optional debug flag before use in reader()
debug_lines_enabled = False

Fields = ["time_ms", "yaw", "pitch", "roll", "yaw_err", "mode", "left", "right", "cmd"]

class DataBuffer:
    """Rolling time window buffer (device time_ms) with manual time-based trimming."""
    def __init__(self, window_ms: int = 15000, est_rate_hz: int = 40) -> None:
        self.lock = threading.Lock()
        maxlen = int(est_rate_hz * (window_ms/1000.0) * 2)
        self.data: Dict[str, Deque[Any]] = {f: collections.deque(maxlen=maxlen) for f in Fields}
        self.window_ms = window_ms
    def _trim(self, newest_time: int) -> None:
        cutoff = newest_time - self.window_ms
        tdeque = self.data['time_ms']
        if not tdeque:
            return
        while tdeque and tdeque[0] < cutoff:
            for f in Fields:
                self.data[f].popleft()
    def add(self, rec: Dict[str, Any]) -> None:
        with self.lock:
            for f, v in rec.items():
                self.data[f].append(v)
            self._trim(rec['time_ms'])
    def snapshot(self) -> Dict[str, list[Any]]:
        with self.lock:
            return {f: list(self.data[f]) for f in Fields}

def reader(ser: serial.Serial, buf, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if not line:
                continue
            s = line.decode("utf-8", "ignore").strip()
        except Exception:
            continue
        if not s:
            continue
        if s.startswith('DATA_HEADER'):
            continue
        if not s.startswith('DATA,'):
            continue
        parts = s.split(',')
        rec = None
        if len(parts) == 10:
            # New format with cmd
            try:
                rec = {
                    "time_ms": int(parts[1]),
                    "yaw": float(parts[2]),
                    "pitch": float(parts[3]),
                    "roll": float(parts[4]),
                    "yaw_err": float(parts[5]),
                    "mode": parts[6],
                    "left": float(parts[7]),
                    "right": float(parts[8]),
                    "cmd": parts[9],
                }
            except ValueError:
                rec = None
        elif len(parts) == 9:
            # Legacy (no cmd); synthesize '-'
            try:
                rec = {
                    "time_ms": int(parts[1]),
                    "yaw": float(parts[2]),
                    "pitch": float(parts[3]),
                    "roll": float(parts[4]),
                    "yaw_err": float(parts[5]),
                    "mode": parts[6],
                    "left": float(parts[7]),
                    "right": float(parts[8]),
                    "cmd": "-",
                }
            except ValueError:
                rec = None
        else:
            # Unexpected length; optionally debug
            if debug_lines_enabled:
                print("SKIP(len=", len(parts), "):", s)
            continue
        if rec is None:
            if debug_lines_enabled:
                print("PARSE_FAIL:", s)
            continue
        buf.add(rec)

def tx_input_thread(ser: serial.Serial, stop_event: threading.Event):
    print("Type drive commands (f,w,b,a,d,x,z,etc). 'quit' to exit. Ctrl+C to save & exit plot.")
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
        except KeyboardInterrupt:
            break
        if not line:
            # EOF
            break
        cmd = line.strip()
        if not cmd:
            continue
        if cmd.lower() == 'quit':
            stop_event.set()
            break
        try:
            ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            print('Write failed:', e, file=sys.stderr)
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True, help='Serial port, e.g. /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=15.0, help='Time window to display (seconds)')
    ap.add_argument('--dump', default='capture.csv', help='CSV file to dump buffer on Ctrl+C (first interrupt)')
    ap.add_argument('--no-tx', action='store_true', help='Disable interactive command sending')
    ap.add_argument('--no-keys', action='store_true', help='Disable figure window key bindings')
    ap.add_argument(
        "--debug-lines",
        action="store_true",
        help="Print skipped / parse-failed DATA lines",
    )
    args = ap.parse_args()

    global debug_lines_enabled
    debug_lines_enabled = args.debug_lines

    buf = DataBuffer(window_ms=int(args.seconds * 1000))
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print('Could not open serial:', e, file=sys.stderr)
        return

    stop_event = threading.Event()
    t_r = threading.Thread(target=reader, args=(ser, buf, stop_event), daemon=True)
    t_r.start()
    if not args.no_tx:
        t_tx = threading.Thread(target=tx_input_thread, args=(ser, stop_event), daemon=True)
        t_tx.start()

    interrupted = {'flag': False}

    def dump_and_exit(*_):
        if interrupted['flag']:
            # second Ctrl+C: force exit immediately
            os._exit(130)
        interrupted['flag'] = True
        snap = buf.snapshot()
        try:
            with open(args.dump, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(Fields)
                for i in range(len(snap['time_ms'])):
                    w.writerow([snap[f][i] for f in Fields])
            print(f"\nSaved {len(snap['time_ms'])} samples to {args.dump}. Press Ctrl+C again to quit immediately.")
        except Exception as e:
            print('Error writing dump:', e, file=sys.stderr)
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        plt.close('all')

    signal.signal(signal.SIGINT, dump_and_exit)

    plt.style.use('seaborn-v0_8-darkgrid')
    fig, (ax1, ax2) = plt.subplots(2,1, figsize=(10,7), sharex=True)
    ax1.set_ylabel('Angle (deg)')
    ax2.set_ylabel('Motors')
    ax2.set_xlabel('Time (s)')
    line_yaw, = ax1.plot([], [], label='Yaw')
    line_pitch, = ax1.plot([], [], label='Pitch')
    line_roll, = ax1.plot([], [], label='Roll')
    line_yerr, = ax1.plot([], [], label='YawErr', linestyle='--')
    line_left, = ax2.plot([], [], label='Left')
    line_right, = ax2.plot([], [], label='Right')
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')

    key_map = {
        'w': 'f',   # forward
        'f': 'f',
        'b': 'b',   # backward
        'x': 'x',   # stop
        's': 'x',   # stop alternative
        'a': 'a',   # turn left
        'd': 'd',   # turn right
        'z': 'z',   # zero yaw
        'p': 'p',   # print help from device
        'q': 'x'    # stop (q) then user can close
    }

    if not args.no_keys:
        print("Key bindings active in plot window: " + ', '.join([f"{k}->{v}" for k,v in key_map.items()]))
        def on_key(event):
            if event.key is None:
                return
            k = event.key.lower()
            if k in key_map:
                cmd = key_map[k]
                try:
                    ser.write((cmd + '\n').encode('utf-8'))
                    print(f"[key:{k}] -> '{cmd}' sent")
                except Exception as e:
                    print('Key send failed:', e, file=sys.stderr)
            # allow quick quit: capital Q closes figure after stop
            if k == 'q':
                # send stop already mapped; also close figure
                plt.close(fig)
        fig.canvas.mpl_connect('key_press_event', on_key)

    def animate(_):
        snap = buf.snapshot()
        if not snap['time_ms']:
            return line_yaw, line_pitch, line_roll, line_yerr, line_left, line_right
        tsec = [(x - snap['time_ms'][0])/1000.0 for x in snap['time_ms']]
        line_yaw.set_data(tsec, snap['yaw'])
        line_pitch.set_data(tsec, snap['pitch'])
        line_roll.set_data(tsec, snap['roll'])
        line_yerr.set_data(tsec, snap['yaw_err'])
        line_left.set_data(tsec, snap['left'])
        line_right.set_data(tsec, snap['right'])
        # Rescale within configured seconds window
        if tsec:
            xmax = tsec[-1]
            xmin = max(0.0, xmax - args.seconds)
            ax1.set_xlim(xmin, xmax)
            ax2.set_xlim(xmin, xmax)
        ax1.relim(); ax1.autoscale_view()
        ax2.relim(); ax2.autoscale_view()
        # Update title with last command if any
        if snap["cmd"]:
            fig.suptitle(f"Last cmd: {snap['cmd'][-1]}")
        return line_yaw, line_pitch, line_roll, line_yerr, line_left, line_right

    # Keep a reference so it is not garbage collected
    ani = animation.FuncAnimation(fig, animate, interval=100, blit=False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
