import pickle
import sys
import time
import logging

import numpy as np
import matplotlib.pyplot as plt
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import math
from fpvModul import CrazyflieFPV
from PyQt6 import QtWidgets
import generate_trajectory
from final import final
import tkinter as tk
import threading
from imagesaver import create_downloader

image_downloader = None
download_thread = None



# samo ispisi greske
logging.basicConfig(level=logging.INFO)


# ova funkcija ostaje vama za implementaciju, dron tijekom tih par sekundi
# treba slikavati korisničke pokrete
def zapocni_snimanje():
    
    global image_downloader, download_thread
    try:
        # Kreiraj downloader i pozovi downloader.run
        image_downloader = create_downloader('tcp://192.168.4.1:5000')
        download_thread = threading.Thread(target=image_downloader.run)
        print("Snimanje zapocinje za 3...")
        time.sleep(1)
        print("2...")
        time.sleep(1)
        print("1...")
        time.sleep(1)
        print("Snimanje započeto!")
        download_thread.start()
        
    except Exception as e:
        print(f"Greška pri pokretanju snimanja: {e}")


# kad se ovo klikne, sve slike slikane cf kamerom bi trebale biti spremljene i 
# u final.py u images bi trebala bit stavljena njihova imena
def zaustavi_snimanje():
    global image_downloader, download_thread
    # ugasi image downloader
    if image_downloader:
        image_downloader.stop()
        if download_thread:
            download_thread.join()
        print("Snimanje zaustavljeno!\nZapočinjem analizu snimljenih slika...")
        # Analiza slika
        final.start_analysis()
    else:
        print("Snimanje nije pokrenuto!")
    
        
def pogledaj_kameru():
    # pokreni FPV modul koji prikazuje stanje kamere
    app = QtWidgets.QApplication([])
    window = CrazyflieFPV()
    window.show()
    app.exec()


def prikazi_pocetne_gumbe():
    # Ukloni sve widgete iz glavnog prozora
    for widget in root.winfo_children():
        widget.destroy()

    # Frame za izbornik
    menu_frame = tk.Frame(root)
    menu_frame.pack(pady=20, padx=20)

    # Gumb za započinjanje snimanja
    start_button = tk.Button(
        menu_frame, text="Započni snimanje", command=zapocni_snimanje,
        width=20, height=2, bg="green", fg="white"
    )
    start_button.pack(side="left", padx=10)

    # Gumb za zaustavljanje snimanja
    stop_button = tk.Button(
        menu_frame, text="Zaustavi snimanje", command=zaustavi_snimanje,
        width=20, height=2, bg="red", fg="white"
    )
    stop_button.pack(side="left", padx=10)

    # Gumb za pogled na kameru
    camera_button = tk.Button(
        menu_frame, text="Pogledaj kameru", command=pogledaj_kameru,
        width=20, height=2, bg="blue", fg="white"
    )
    camera_button.pack(side="left", padx=10)



if __name__ == '__main__':

    root = tk.Tk()
    root.title("GUI za izvođenje korisnikove trajektorije")

    #Prikaz početnih gumba
    prikazi_pocetne_gumbe()
    
    # Pokreni aplikaciju
    root.mainloop()
    
    
    
