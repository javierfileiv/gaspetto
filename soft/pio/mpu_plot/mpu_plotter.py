"""Unified plotter wrapper (copies improved version from project root)."""
import argparse
import csv
import queue
import re
import sys
import threading
from typing import Deque, Dict, List, Optional, Tuple
import time
"""Unified plotter wrapper (copies improved version from project root).

Dependency note:
    Requires pyserial (package name: pyserial) and matplotlib. If you see
    AttributeError: module 'serial' has no attribute 'Serial'
    it usually means either:
        1) pyserial isn't installed (pip install pyserial), or
        2) You accidentally installed the wrong package named 'serial', or
        3) A local file/folder named 'serial.py' or 'serial/' shadows pyserial.
    This module now performs a runtime check and prints a helpful message.
"""
try:
        import serial  # pyserial
        try:  # Validate expected attributes exist
                from serial import Serial, SerialException  # type: ignore
        except Exception:  # pragma: no cover - defensive
                Serial = None  # type: ignore
                SerialException = Exception  # type: ignore
except ImportError as _e:  # pragma: no cover
        serial = None  # type: ignore
        Serial = None  # type: ignore
        SerialException = Exception  # type: ignore
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

DEFAULT_SERIAL_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_MAX_POINTS = 400
MIN_DATA_VALUES = 9
MIN_TOTAL_VALUES = 11
FLOAT_TOKEN = re.compile(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")

args = None
accel_x_data: Optional[Deque[float]] = None
accel_y_data: Optional[Deque[float]] = None
accel_z_data: Optional[Deque[float]] = None
gyro_x_data: Optional[Deque[float]] = None
gyro_y_data: Optional[Deque[float]] = None
gyro_z_data: Optional[Deque[float]] = None
kalman_roll_data: Optional[Deque[float]] = None
kalman_pitch_data: Optional[Deque[float]] = None
integrated_yaw_data: Optional[Deque[float]] = None
time_data: Optional[Deque[float]] = None
line_queue: "queue.Queue[str]" = queue.Queue(maxsize=2000)
stop_event = threading.Event()
pause_event = threading.Event()
stats: Dict[str, float] = {
    'lines_total': 0,
    'lines_parsed': 0,
    'last_update_time': time.time(),
    'fps': 0.0,
    'sps': 0.0,
    'seq_last': -1,
    'seq_gaps': 0,
    'seq_resets': 0
}
csv_writer = None
csv_file_handle = None

def parse_args():
    p = argparse.ArgumentParser(description='Live plot (Seq,Micros + 9 values).')
    p.add_argument('-p','--port', default=DEFAULT_SERIAL_PORT)
    p.add_argument('-b','--baud', type=int, default=DEFAULT_BAUD_RATE)
    p.add_argument('-n','--points', type=int, default=DEFAULT_MAX_POINTS)
    p.add_argument('-i','--interval', type=int, default=50, help='Update interval ms')
    p.add_argument('-s','--save', help='CSV log file')
    p.add_argument('--max-wait-header', type=float, default=5.0, help='Seconds to wait for header before fallback')
    p.add_argument('--no-header-wait', action='store_true', help='Skip waiting for header line')
    p.add_argument('--debug', action='store_true', help='Print each raw line and first few parsed sets')
    p.add_argument('--window-seconds', type=float, default=15.0, help='Visible rolling time window (seconds)')
    return p.parse_args()

def open_serial(port: str, baud: int):
    if serial is None or Serial is None:
        print('[FATAL] pyserial not available. Install with: pip install pyserial')
        print('[HINT ] If you installed a package named "serial", uninstall it and install "pyserial".')
        sys.exit(2)
    try:
        s = Serial(port, baud, timeout=0.2)
        print(f'[INFO] Opened {port} @ {baud}')
        return s
    except SerialException as e:
        print(f'[ERROR] {e}')
        sys.exit(1)

def wait_header(s, timeout):
    if args.no_header_wait:
        print('[INFO] Skipping header wait (--no-header-wait)')
        return False
    start=time.time(); printed=0
    while time.time()-start < timeout and not stop_event.is_set():
        line=s.readline().decode('utf-8','replace').strip()
        if not line:
            continue
        if 'Streaming Seq' in line or line.startswith('Streaming Acc') or line.startswith('Streaming ') or 'AccX' in line:
            print('[INFO] Header:', line)
            return True
        if printed < 5:
            print('[DBG pre-header]', line)
            printed += 1
    print('[WARN] No header within timeout; continuing anyway')
    return False

def reader_thread(s):
    while not stop_event.is_set():
        try:
            raw=s.readline()
            if not raw: continue
            dec=raw.decode('utf-8','replace').strip()
            if dec:
                if args and args.debug:
                    print('[RAW]', dec)
                try: line_queue.put_nowait(dec)
                except queue.Full:
                    while not line_queue.empty():
                        try: line_queue.get_nowait()
                        except queue.Empty: break
        except Exception:
            break
    try: s.close()
    except: pass

def init(points):
    global accel_x_data,accel_y_data,accel_z_data,gyro_x_data,gyro_y_data,gyro_z_data
    global kalman_roll_data,kalman_pitch_data,integrated_yaw_data,time_data
    accel_x_data=deque(maxlen=points)
    accel_y_data=deque(maxlen=points)
    accel_z_data=deque(maxlen=points)
    gyro_x_data=deque(maxlen=points)
    gyro_y_data=deque(maxlen=points)
    gyro_z_data=deque(maxlen=points)
    kalman_roll_data=deque(maxlen=points)
    kalman_pitch_data=deque(maxlen=points)
    integrated_yaw_data=deque(maxlen=points)
    time_data=deque(maxlen=points)

def parse_line(line:str):
    if line.count(',') < MIN_DATA_VALUES -1: return None
    parts=[p for p in line.split(',') if p.strip()!='']
    if len(parts)<MIN_DATA_VALUES: return None
    seq=micros=None
    try:
        if len(parts)>=MIN_TOTAL_VALUES:
            seq=int(float(parts[0])); micros=int(float(parts[1]))
            data=[float(x) for x in parts[-MIN_DATA_VALUES:]]
        else:
            data=[float(x) for x in parts[-MIN_DATA_VALUES:]]
    except ValueError:
        return None
    if len(data)!=MIN_DATA_VALUES: return None
    return seq,micros,data

early_parsed_count = 0
def consume():
    global early_parsed_count
    parsed=None
    for _ in range(200):
        try: line=line_queue.get_nowait()
        except queue.Empty: break
        stats['lines_total']+=1
        p=parse_line(line)
        if p:
            parsed=p; stats['lines_parsed']+=1
            if args and args.debug and early_parsed_count < 5:
                seq, micros, data = p
                print(f"[PARSED {early_parsed_count}] seq={seq} micros={micros} data={data}")
                early_parsed_count += 1
    return parsed

def setup_plot():
    fig,axes=plt.subplots(3,1,figsize=(10,12))
    fig.suptitle('MPU6050 Live Data (Seq,Micros + 9 values) [space=pause q=quit]')
    ax1=axes[0]; ax2=axes[1]; ax3=axes[2]
    l1,=ax1.plot([],[],label='AccX'); l2,=ax1.plot([],[],label='AccY'); l3,=ax1.plot([],[],label='AccZ')
    ax1.legend(); ax1.set_ylabel('m/s^2'); ax1.grid(True)
    g1,=ax2.plot([],[],label='GyroX'); g2,=ax2.plot([],[],label='GyroY'); g3,=ax2.plot([],[],label='GyroZ')
    ax2.legend(); ax2.set_ylabel('rad/s'); ax2.grid(True)
    k1,=ax3.plot([],[],label='Roll'); k2,=ax3.plot([],[],label='Pitch'); k3,=ax3.plot([],[],label='Yaw')
    ax3.legend(); ax3.set_ylabel('deg'); ax3.set_xlabel('t (s)'); ax3.set_ylim(-185,185); ax3.grid(True)
    fig.tight_layout(rect=[0,0.03,1,0.95])
    return fig,(ax1,ax2,ax3),(l1,l2,l3,g1,g2,g3,k1,k2,k3)

def dyn_ylim(ax,series):
    allv=[]
    for s in series:
        if s: allv.extend(s)
    if not allv: return
    mn=min(allv); mx=max(allv)
    if mn==mx: ax.set_ylim(mn-0.5,mx+0.5)
    else:
        m=(mx-mn)*0.1; ax.set_ylim(mn-m,mx+m)

def on_key(evt):
    if evt.key==' ':
        if pause_event.is_set(): pause_event.clear(); print('[INFO] Resumed')
        else: pause_event.set(); print('[INFO] Paused')
    elif evt.key=='q': stop_event.set(); plt.close(evt.canvas.figure)

start_time=None
window_seconds=15.0

def animate(_):
    global start_time
    if pause_event.is_set(): return lines
    parsed=consume()
    if not parsed: return lines
    seq,micros,data=parsed
    now=time.time()
    t=now-start_time
    time_data.append(t)
    accel_x_data.append(data[0]); accel_y_data.append(data[1]); accel_z_data.append(data[2])
    gyro_x_data.append(data[3]); gyro_y_data.append(data[4]); gyro_z_data.append(data[5])
    kalman_roll_data.append(data[6]); kalman_pitch_data.append(data[7]); integrated_yaw_data.append(data[8])
    # Trim old samples outside window
    cutoff = t - window_seconds
    while time_data and time_data[0] < cutoff:
        time_data.popleft()
        accel_x_data.popleft(); accel_y_data.popleft(); accel_z_data.popleft()
        gyro_x_data.popleft(); gyro_y_data.popleft(); gyro_z_data.popleft()
        kalman_roll_data.popleft(); kalman_pitch_data.popleft(); integrated_yaw_data.popleft()
    xmin=max(0,time_data[0]); xmax=time_data[-1]+0.2
    ax1.set_xlim(xmin,xmax); ax2.set_xlim(xmin,xmax); ax3.set_xlim(xmin,xmax)
    dyn_ylim(ax1,[accel_x_data,accel_y_data,accel_z_data])
    dyn_ylim(ax2,[gyro_x_data,gyro_y_data,gyro_z_data])
    l1,l2,l3,g1,g2,g3,k1,k2,k3=lines
    l1.set_data(time_data,accel_x_data); l2.set_data(time_data,accel_y_data); l3.set_data(time_data,accel_z_data)
    g1.set_data(time_data,gyro_x_data); g2.set_data(time_data,gyro_y_data); g3.set_data(time_data,gyro_z_data)
    k1.set_data(time_data,kalman_roll_data); k2.set_data(time_data,kalman_pitch_data); k3.set_data(time_data,integrated_yaw_data)
    return lines

def main():
    global args,fig,ax1,ax2,ax3,lines,start_time
    args=parse_args();
    global window_seconds
    window_seconds = args.window_seconds
    init(args.points); s=open_serial(args.port,args.baud); wait_header(s,args.max_wait_header)
    t=threading.Thread(target=reader_thread,args=(s,),daemon=True); t.start()
    fig,(ax1,ax2,ax3),lines=setup_plot(); fig.canvas.mpl_connect('key_press_event',on_key)
    start_time=time.time()
    global anim
    anim = animation.FuncAnimation(fig, animate, interval=args.interval, blit=False, cache_frame_data=False)
    # Console stats updater thread
    def stats_printer():
        last_parsed = 0
        last_time = time.time()
        while not stop_event.is_set():
            time.sleep(1.0)
            parsed_now = stats['lines_parsed']
            now = time.time()
            rate = (parsed_now - last_parsed) / (now - last_time) if now > last_time else 0
            last_parsed = parsed_now
            last_time = now
            print(f"[STAT] parsed_total={parsed_now} recent_rate={rate:.1f} Hz queue={line_queue.qsize()} gaps={int(stats['seq_gaps'])} resets={int(stats['seq_resets'])}")
    threading.Thread(target=stats_printer, daemon=True).start()
    try: plt.show()
    except KeyboardInterrupt: pass
    stop_event.set()
    print('[INFO] Exit')

if __name__=='__main__':
    main()
