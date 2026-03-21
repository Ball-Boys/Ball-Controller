"""
OTA firmware upload script for ESP32 Ball Controller.

Uploads a firmware binary to the ESP32 via HTTP POST.
The ESP32 must be running and connected (WiFi AP mode).

Usage:
    python ota_upload.py [firmware_path] [--ip ESP_IP]

If firmware_path is omitted, it looks for the PlatformIO build output at:
    ../esp-controller-idf/.pio/build/esp32dev/firmware.bin
"""

import argparse
import os
import sys
import urllib.request
import urllib.error

DEFAULT_FIRMWARE_PATH = os.path.join(
    os.path.dirname(__file__),
    "..", "esp-controller-idf", ".pio", "build", "esp32dev", "firmware.bin"
)
DEFAULT_ESP_IP = "192.168.4.1"
OTA_PORT = 8080


def upload_firmware(firmware_path: str, esp_ip: str) -> bool:
    firmware_path = os.path.abspath(firmware_path)
    if not os.path.isfile(firmware_path):
        print(f"ERROR: Firmware file not found: {firmware_path}")
        return False

    file_size = os.path.getsize(firmware_path)
    print(f"Firmware: {firmware_path} ({file_size:,} bytes)")

    url = f"http://{esp_ip}:{OTA_PORT}/update"
    print(f"Uploading to {url} ...")

    with open(firmware_path, "rb") as f:
        data = f.read()

    req = urllib.request.Request(
        url,
        data=data,
        headers={
            "Content-Type": "application/octet-stream",
            "Content-Length": str(len(data)),
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=120) as resp:
            body = resp.read().decode("utf-8", errors="replace")
            print(f"Response ({resp.status}): {body}")
            return resp.status == 200
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        print(f"HTTP Error {e.code}: {body}")
        return False
    except urllib.error.URLError as e:
        print(f"Connection error: {e.reason}")
        print("Make sure you are connected to the ESP32 WiFi network.")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Upload firmware to ESP32 via OTA")
    parser.add_argument("firmware", nargs="?", default=DEFAULT_FIRMWARE_PATH,
                        help="Path to firmware.bin (default: PlatformIO build output)")
    parser.add_argument("--ip", default=DEFAULT_ESP_IP,
                        help=f"ESP32 IP address (default: {DEFAULT_ESP_IP})")
    args = parser.parse_args()

    success = upload_firmware(args.firmware, args.ip)
    sys.exit(0 if success else 1)
