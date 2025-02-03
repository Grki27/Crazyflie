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
import generate_trajectory
from final import final
import tkinter as tk
import threading

# Postavi URI za Crazyflie
uri = uri_helper.uri_from_env(default="radio://0/80/2M")

# Inicijalizacija cflib biblioteke
cflib.crtp.init_drivers(enable_debug_driver=False)

# Callback za primanje log podataka
def log_data_callback(timestamp, data, logconf):
    print(f"Timestamp: {timestamp}, Data: {data}")

# Glavna funkcija
def main():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Povezano s Crazyflie!")

        # Konfiguracija za logiranje podataka s Flowdeck-a
        log_config = LogConfig(name="Flowdeck", period_in_ms=100)
        log_config.add_variable("range.zrange", "uint16_t")  # Visina
        log_config.add_variable("motion.deltaX", "float")    # Optički tok X
        log_config.add_variable("motion.deltaY", "float")    # Optički tok Y

        try:
            scf.cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(log_data_callback)
            log_config.start()

            print("Logiranje Flowdeck podataka...")
            time.sleep(5)  # Logiraj 5 sekundi
            log_config.stop()
        except KeyError as e:
            print(f"Greška: Nije moguće pronaći varijablu. Flowdeck možda nije povezan: {e}")
        except AttributeError:
            print("Greška: Flowdeck nije povezan ili nije konfiguriran pravilno.")

if __name__ == "__main__":
    main()
