#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging
import shutil
import struct
import time
import os
import threading

import numpy as np
import cv2
import cflib
from cflib.cpx import CPXFunction

logging.basicConfig(level=logging.INFO)

# Dimenzije kamere
CAM_HEIGHT = 244
CAM_WIDTH = 324


class ImageDownloader:
    def __init__(self, cpx):
        self._cpx = cpx
        threading.Thread.__init__(self)
        self.daemon = True
        self.image_counter = 0
        self.output_dir = "images"
        self.stop_event = threading.Event()  # Signal za zaustavljanje

        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)  # Briše cijeli folder i sve unutar njega

        os.makedirs(self.output_dir) 

    # funkcija za slikavanje slika  
    def run(self):
        print("Image download started.")
        while not self.stop_event.is_set():
            print("Čekam paket...")
            try:
                # dobavi podatke
                packet = self._cpx.receivePacket(CPXFunction.APP)
                if not packet or len(packet.data) < 11:
                    print("⚠️  Nema podataka, čekam...")
                    continue
            
                [magic, width, height, depth, fmt, size] = struct.unpack('<BHHBBI', packet.data[0:11])
                if magic != 0xBC:
                    logging.warning("Invalid packet received")
                    continue
                print("dobiven validan paket")
                img_data = bytearray()
                while len(img_data) < size:
                    packet = self._cpx.receivePacket(CPXFunction.APP)
                    if packet:
                        img_data.extend(packet.data)

                self.save_image_as_png(img_data)
                # slikavanje svako 0.3 sekunde
                time.sleep(0.1)

            except BrokenPipeError:
                print("❌ Izgubljena veza! Pokušavam ponovo...")
                self.reconnect() 

        logging.info("Image download stopped.")

    def save_image_as_png(self, img_data):
        bayer_img = np.frombuffer(img_data, dtype=np.uint8).reshape((CAM_HEIGHT, CAM_WIDTH))
        filename = os.path.join(self.output_dir, f"image_{self.image_counter:04d}.png")
        cv2.imwrite(filename, bayer_img)
        logging.info(f"PNG Image saved: {filename}")
        self.image_counter += 1

    def stop(self):
        logging.info("Stopping image download...")
        self.stop_event.set()

    def reconnect(self):
        print("🔄 Pokušavam ponovo povezivanje...")
        self._cpx.close()  # Zatvori staru vezu
        time.sleep(2)  # Pričekaj malo
        self._cpx.open()  # Pokušaj ponovo otvoriti konekciju
        print("✅ Ponovno povezan!")

def create_downloader(uri):
    """Povezuje se na dron i vraća instancu ImageDownloader."""
    cflib.crtp.init_drivers()
    cf = cflib.crazyflie.Crazyflie(ro_cache=None, rw_cache="cache")
    cf.open_link(uri)

    if not cf.link or not hasattr(cf.link, "cpx"):
        raise ConnectionError("Connection failed. Ensure the drone is connected over WiFi.")
    
    return ImageDownloader(cf.link.cpx)
