#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import struct
import sys
import threading
import time
import numpy as np
import cflib.crtp
from cflib.cpx import CPXFunction
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6.QtGui import QImage


CAM_HEIGHT = 244
CAM_WIDTH = 324
SPEED_FACTOR = 0.3


class ImageDownloader(threading.Thread):
    def __init__(self, cpx, callback):
        threading.Thread.__init__(self)
        self.daemon = True
        self._cpx = cpx
        self._callback = callback
        self.running = True  # Flag za prekid threada

    def run(self):
        while self.running:  # Provjera flag-a
            packet = self._cpx.receivePacket(CPXFunction.APP)
            if packet and len(packet.data) >= 11:
                [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', packet.data[0:11])
                if magic == 0xBC:
                    imgStream = bytearray()
                    while len(imgStream) < size and self.running:  # Provjera da li treba stati
                        packet = self._cpx.receivePacket(CPXFunction.APP)
                        imgStream.extend(packet.data)

                    if self.running:  # Dodatna provjera prije callback-a
                        image = np.frombuffer(imgStream, dtype=np.uint8)
                        self._callback(image)
                    
                    time.sleep(0.1)

    def stop(self):
        self.running = False  # Zaustavi thread


class CrazyflieFPV(QtWidgets.QWidget):
    def __init__(self, uri='tcp://192.168.4.1:5000'):
        super().__init__()

        self.setWindowTitle('Crazyflie / AI deck FPV')
        self.layout = QtWidgets.QVBoxLayout()
        self.image_frame = QtWidgets.QLabel()
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        self.cf.open_link(uri)

        if not hasattr(self.cf.link, 'cpx'):
            print('Not connecting with WiFi')
            self.cf.close_link()
        else:
            self.img_downloader = ImageDownloader(self.cf.link.cpx, self.update_image)
            self.img_downloader.start()

    def update_image(self, image):
        try:
            img = QtGui.QImage(image, CAM_WIDTH, CAM_HEIGHT, QtGui.QImage.Format.Format_Grayscale8)
            img = img.scaled(648, 488)
            self.image_frame.setPixmap(QtGui.QPixmap.fromImage(img))
        except Exception as e:
            print("Error updating image:", e)

    def connected(self, uri):
        print(f'Connected to {uri}')

    def pos_data(self, timestamp, data, logconf):
        print(f'Position Data: {data}')

    def disconnected(self, uri):
        print('Disconnected')

    def closeEvent(self, event):
        print("Zatvaranje aplikacije...")
        
        # Zaustavi downloader thread
        if hasattr(self, 'img_downloader') and self.img_downloader:
            self.img_downloader.stop()
            self.img_downloader.join()  # Pričekaj da se thread ugasi

        # Zatvori vezu s dronom
        if self.cf is not None:
            self.cf.close_link()
            time.sleep(1)

        event.accept()  # Nastavi zatvaranje aplikacije
