import ctypes
import subprocess
import time

from arm import STS3215, grab, release, arm_init
from motor import Motor, forward, bread

BLACK_THRESHOLD = 80
BLACK_RATIO = 0.5
SPEED = 240

UART_DEV = "/dev/ttyS3"
UART_BAUD = 1500000
UART_TIMEOUT_MS = 2000
FRAME_BUF_SIZE = 2 * 1024 * 1024
FRAME_PATH = "esp_frame.jpg"

lib = ctypes.CDLL("/root/AKA-00/libuart_capture.so")
lib.uart_cap_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
lib.uart_cap_open.restype = ctypes.c_void_p
lib.uart_cap_get_frame.argtypes = [
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32),
]
lib.uart_cap_get_frame.restype = ctypes.c_int
lib.uart_cap_close.argtypes = [ctypes.c_void_p]
lib.uart_cap_close.restype = None

frame_buf = (ctypes.c_uint8 * FRAME_BUF_SIZE)()
frame_len = ctypes.c_uint32(0)


def get_frame(handle):
    ret = lib.uart_cap_get_frame(handle, frame_buf, FRAME_BUF_SIZE,
                                 ctypes.byref(frame_len))
    if ret != 0:
        raise RuntimeError(f"uart_cap_get_frame failed: {ret}")
    n = frame_len.value
    return bytes(frame_buf[:n])


def is_mostly_black(jpeg_data, threshold=BLACK_THRESHOLD, ratio=BLACK_RATIO):
    try:
        from PIL import Image
        import io
        img = Image.open(io.BytesIO(jpeg_data)).convert("L")
        hist = img.histogram()
        black = sum(hist[:threshold])
        total = img.size[0] * img.size[1]
        return black / total > ratio
    except Exception:
        pass

    # tmp = "/tmp/_frame_check.jpg"
    # with open(tmp, "wb") as f:
    #     f.write(jpeg_data)
    try:
        raw = subprocess.check_output(["djpeg", "-grayscale", "-pnm", tmp])
        idx = 0
        for _ in range(3):
            idx = raw.index(b'\n', idx) + 1
        pixels = raw[idx:]
        black = sum(1 for b in pixels if b < threshold)
        return black / len(pixels) > ratio
    except Exception:
        avg = sum(jpeg_data) / len(jpeg_data)
        return avg < 100


left_motor = Motor(4, 0, 1)
right_motor = Motor(4, 2, 3)
servo = STS3215("/dev/ttyS2", baudrate=115200)
arm_init(servo)

handle = lib.uart_cap_open(UART_DEV.encode(), UART_BAUD, UART_TIMEOUT_MS)
if not handle:
    raise RuntimeError("uart_cap_open failed")

forward(left_motor, right_motor, SPEED)
time.sleep(1)
bread(left_motor, right_motor)

try:
    while True:
        try:
            print("[*] capturing frame ...")
            jpeg = get_frame(handle)
            # with open(FRAME_PATH, "wb") as f:
            #     f.write(jpeg)
            print(f"[*] got {len(jpeg)} bytes")

            if is_mostly_black(jpeg, BLACK_THRESHOLD, BLACK_RATIO):
                print("[*] black dominant -> forward & grab")
                forward(left_motor, right_motor, SPEED)
                time.sleep(1)
                bread(left_motor, right_motor)
                grab(servo)
            else:
                print("[*] not black dominant -> stop")
                bread(left_motor, right_motor)
        except Exception as e:
            print(f"[error] {e}")
            bread(left_motor, right_motor)
            break
except KeyboardInterrupt:
    print("\n[exit] interrupted")
finally:
    bread(left_motor, right_motor)
    lib.uart_cap_close(handle)
