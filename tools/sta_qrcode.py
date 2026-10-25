#!/usr/bin/env python3
"""Show the QR code that puts the RoboMaster EP into router (STA) mode on a given Wi-Fi network.

Usage (venv active):  python3 tools/sta_qrcode.py [--ssid "Z&M"]
- Asks for the Wi-Fi password without echoing it. The password is never printed or saved.
- Shows the QR code in a window: hold the screen 20-30 cm in front of the robot camera
  (robot switch on the RIGHT, router mode, after the voice asks to scan the code).
- Then waits for the robot to announce itself on the network and prints its IP. Press q in the
  window or Ctrl+C here to quit.
"""
import argparse
import getpass
import threading

import cv2
import numpy as np
import qrcode
from robomaster import conn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ssid', default='Z&M')
    args = ap.parse_args()
    password = getpass.getpass(f'Wi-Fi password for "{args.ssid}" (not shown): ')

    helper = conn.ConnectionHelper()
    payload = helper.build_qrcode_string(ssid=args.ssid, password=password)
    del password

    qr = qrcode.QRCode(border=4, box_size=1)
    qr.add_data(payload)
    qr.make(fit=True)
    mat = np.array(qr.get_matrix(), dtype=np.uint8)
    img = np.where(mat, 0, 255).astype(np.uint8)
    img = cv2.resize(img, (600, 600), interpolation=cv2.INTER_NEAREST)

    def wait_robot():
        if helper.wait_for_connection():
            print('Robot joined the network and answered. You can close the QR window.')
    threading.Thread(target=wait_robot, daemon=True).start()

    print(f'Showing QR for SSID "{args.ssid}". Hold it in front of the robot camera.')
    while True:
        cv2.imshow('RoboMaster router-mode QR (press q to quit)', img)
        if cv2.waitKey(200) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
