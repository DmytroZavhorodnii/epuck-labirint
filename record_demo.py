"""Record Webots simulation as GIF using screenshots."""
import subprocess
import time
import sys
import os
from PIL import ImageGrab
import win32gui
import win32con

WORLD_PATH = r"c:\Users\Administrator\Desktop\labirynt\worlds\labirynt.wbt"
OUTPUT_GIF = r"c:\Users\Administrator\Desktop\labirynt\images\demo.gif"
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
DURATION = 15  # seconds
FPS = 10
FRAME_INTERVAL = 1.0 / FPS

def find_webots_window():
    """Find the Webots 3D view window."""
    def callback(hwnd, hwnds):
        if win32gui.IsWindowVisible(hwnd):
            title = win32gui.GetWindowText(hwnd)
            if "Labirynt" in title or "Webots" in title:
                hwnds.append((hwnd, title))
        return True

    hwnds = []
    win32gui.EnumWindows(callback, hwnds)
    # Prefer the rendering window (SciWindow or similar)
    for hwnd, title in hwnds:
        if "Webots" in title:
            return hwnd
    return None


def main():
    print("Starting Webots...")
    proc = subprocess.Popen(
        [WEBOTS_EXE, "--mode=fast", WORLD_PATH],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # Wait for Webots to start and initialize
    print("Waiting for Webots to initialize...")
    time.sleep(5)

    # Take screenshots
    frames = []
    start_time = time.time()
    frame_count = 0

    while time.time() - start_time < DURATION:
        loop_start = time.time()

        # Take screenshot of the whole screen
        # (Webots takes most of the screen, so fullscreen grab works fine)
        img = ImageGrab.grab()
        frames.append(img)
        frame_count += 1
        print(f"Frame {frame_count} captured ({time.time() - start_time:.1f}s)")

        elapsed = time.time() - loop_start
        sleep_time = max(0, FRAME_INTERVAL - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Save as GIF
    print(f"Saving {len(frames)} frames to {OUTPUT_GIF}...")
    if frames:
        frames[0].save(
            OUTPUT_GIF,
            save_all=True,
            append_images=frames[1:],
            duration=int(1000 / FPS),
            loop=0,
            optimize=True,
        )
        print(f"Saved: {OUTPUT_GIF} ({os.path.getsize(OUTPUT_GIF) / 1024:.0f} KB)")

    # Kill Webots
    print("Closing Webots...")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()

    print("Done!")


if __name__ == "__main__":
    main()
