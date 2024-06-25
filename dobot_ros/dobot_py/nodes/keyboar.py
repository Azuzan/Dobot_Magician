#!/usr/bin/env python
from __future__ import print_function

from dobot_py import DobotClient as dc
from dobot_py.jogMode import JOGMode
import tkinter as tk

#from dobot_py import DobotClient as dc
#from dobot_py.jogMode import JOGMode
from pynput import keyboard

# Inisialisasi koneksi dengan Dobot
dobot = dc()

# Fungsi untuk mengirim perintah gerakan ke Dobot
def send_command(key):
    # Mapping antara tombol keyboard dan perintah gerakan
    if key == keyboard.Key.up:
        dobot.set_jog_cmd(False, JOGMode.FOWARD.value)
    elif key == keyboard.Key.down:
        dobot.set_jog_cmd(False, JOGMode.BACK.value)
    elif key == keyboard.Key.left:
        dobot.set_jog_cmd(False, JOGMode.LEFT.value)
    elif key == keyboard.Key.right:
        dobot.set_jog_cmd(False, JOGMode.RIGHT.value)

# Fungsi callback saat tombol keyboard ditekan
def on_press(key):
    try:
        # Kirim perintah gerakan jika tombol panah ditekan
        send_command(key)
    except AttributeError:
        pass

# Fungsi callback saat tombol keyboard dilepas
def on_release(key):
    if key == keyboard.Key.esc:
        # Stop program jika tombol escape ditekan
        return False
    else:
        # Stop perintah gerakan saat tombol panah dilepas
        dobot.set_jog_cmd(False, JOGMode.STOP.value)

# Membuat objek listener untuk keyboard
listener = keyboard.Listener(on_press=on_press, on_release=on_release)

if __name__ == "__main__":
    # Memulai koneksi dengan Dobot
    dobot.start_connection()

    # Memulai mendengarkan input keyboard
    listener.start()

    # Menunggu hingga program berhenti dengan menekan tombol escape
    listener.join()

    # Mematikan koneksi dengan Dobot setelah program berakhir
    dobot.close_connection()

