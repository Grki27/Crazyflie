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
from PyQt6 import QtWidgets
import generate_trajectory
import tkinter as tk
from final import final


# samo ispisi greske
logging.basicConfig(level=logging.INFO)

global simplified_coords
simplified_coords = []

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# ovdje kreće nas dio, tj. prilagođavanje početnih točaka + izvođenje putanje


# dodajemo koordinatu visine putanji
def add_x_coordinate_to_points(points, x = 0):
    return [(x, y, z) for y, z in points]



# prilagođavamo dobivene koordinate tako da prva počinje u ishodištu koordinatnog sustava i 
# dodajemo visinu height na kojoj dron leti
def adjust_points_to_start_at_takeoff_point(points, starting_point):
    _, y_start, z_start = starting_point
    return [(x, y - y_start, z - z_start + height) for x, y, z in points]
    
def scale_points(points, scaling_factor):
    return[(y * scaling_factor, z * scaling_factor) for y, z in points]

# ovo je funkcija koja nalazi najmanju točku i dodaje joj visinu kako bi se cijela trajektorija našla iznad tla
def adjust_points_so_drone_is_above_ground(points):
    # Pronađi indeks točke s najmanjom z koordinatom
    min_z  = height
    for i in range(len(points)):
       x, y, z = points[i]
       if (z < min_z):
        min_z = z

    # izračunaj koliku visinu treba dodati svim točkama
    difference = height - min_z 
    print("Difference", difference)

    # dodaj visinu
    return [(x, y, z + difference) for (x, y, z) in points]


#izračunaj ukupnu duljinu čitave trajektorije
def calculate_total_length_of_trajectory(points):
    total_length_of_trajectory = 0.0

    # Petlja kroz sve točke
    for i in range(1, len(points)):
        x, y, z = points[i]
        x_prev, y_prev, z_prev = points[i - 1] # ovo ce za i = 1 biti prilagodena pocetna tocka u ishodistu
        
        # Duljina segmenta između trenutne i prethodne točke
        length_of_current_goto = math.sqrt((x - x_prev)**2 + (y - y_prev)**2 + (z - z_prev)**2)
        total_length_of_trajectory += length_of_current_goto

    return total_length_of_trajectory

# funkcija koja dodaje vrijeme u kojem se dron mora naci u nekoj tocki, kao i yaw (koji je 0)
# ovo je zapravo svojevrsna priprema podataka prije nego što ih pošaljemo u generate_trajectory
def add_time_and_yaw_to_points(absolute_points, timeToExecute, totalLength):
    timeCurrent = 0
    absolute_points_with_time = []
    for i in range(len(absolute_points)):
        x, y, z = absolute_points[i]
        if i > 0:
            x_prev, y_prev, z_prev = absolute_points[i - 1]
            length_of_current_goto = math.sqrt((x - x_prev)**2 + (y - y_prev)**2 + (z - z_prev)**2)
           
            # vrijeme dodajem na principu što je segment dulji, to mu više vremena dajem za izvođenje
            timeCurrent += (length_of_current_goto / totalLength) * timeToExecute
 
        absolute_points_with_time.append((timeCurrent, x, y, z, 0))
    
    return absolute_points_with_time




# funkcija koja uploada svaki redak pripremljene trajektorije u crazyflieovu memoriju, vraća ukupnu duljinu trajektorije
def upload_trajectory(cf, trajectory_id, trajectory, timeToExecute):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    timeToExecute = timeToExecute
    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    # dodajem još vrijeme potrebno da izvede zadnji komad rute
    total_duration += timeToExecute - total_duration

    upload_result = trajectory_mem.write_data_sync()
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

# funkcija koja pokreće izvođenje trajektorije
def run_sequence(cf, trajectory_id, duration, points):
    commander = cf.high_level_commander

    activate_mellinger_controller(cf)
    wait_for_position_estimator(cf)
    commander = cf.high_level_commander
    # sva sleep vremena su relativna, tako samo promjenom visine sve funkcionira glatko
    commander.takeoff(height, height * 5)
    time.sleep(height * 5)
    
    # moram ga dignut na visinu dovoljnu da izvrsi trajektoriju bez da se zabije u pod

    points_with_time = points
    (vrijeme, x, y, z, yaw) = points_with_time[0]
    difference = abs(z - height)

    # dizanje na visinu
    commander.go_to(0, 0, z, 0, difference * 5, relative = False)
    print("\nDrone is flying to: ",z)
    print(" before starting trajectory\n")
    time.sleep(difference * 5)

    # sad kada se nalazi na dovoljnoj visini za izvođenje, trajektorija se pokreće
    relative = False
    commander.start_trajectory(trajectory_id, 1.0, relative)

    # dio koda za bilježenje pozicije drona
    position_log = []
    start_time = time.time()

    # vremena u kojima bi se dron trebao nalaziti u određenim točkama
    target_times = [point[0] for point in points_with_time]
    special_points = []

    # u lg_position konfiguraciju dodajemo x, y i z koordinate
    lg_position = LogConfig(name='Position', period_in_ms = 10)  # Logiranje svakih 10 ms
    lg_position.add_variable('kalman.stateX', 'float')
    lg_position.add_variable('kalman.stateY', 'float')
    lg_position.add_variable('kalman.stateZ', 'float')

    # za vrijeme trajektorije punimo position_log sa koordinatama
    with SyncLogger(cf, lg_position) as logger:
        for log_entry in logger:
            data = log_entry[1]
            timestamp = time.time() - start_time

            position_log.append((data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']))
            print(f"Position: ({data['kalman.stateX']}, {data['kalman.stateY']}, {data['kalman.stateZ']})")

            # special_points su točke u kojima se dron nalazi u zadanim vremenskim trenutcima
            for i, target_time in enumerate(target_times):
                if abs(timestamp - target_time) < 0.05:  # Tolerancija od 100 ms
                    special_points.append((data['kalman.stateY'], data['kalman.stateZ']))
            
            # Prekini logiranje kad je trajektorija završena
            if timestamp > duration + 1:  
                break

    # slijeće toliko dugo koliko se visoko nalazi (da mu damo 2 sekunde a nalazi se na 1.5 metara srušio bi se)
    x_last, y_last, z_last = simplified_coords[len(simplified_coords) - 1]
    commander.land(0.0, z_last * 4)
    print("\nTime to land: ", z_last * 4)
    time.sleep(z_last * 4)
    commander.stop()

    # dobavi sve x, y i z koordinate i nacrtaj ih
    x_positions = [pos[0] for pos in position_log]
    y_positions = [pos[1] for pos in position_log]
    z_positions = [pos[2] for pos in position_log]

    # izračunaj razliku početne i krajnje pozicije
    start_pos = position_log[0]
    end_pos = position_log[-1]
    distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)

    special_y = [point[0] for point in special_points]
    special_z = [point[1] for point in special_points]

    # Ispis udaljenosti
    print(f"Distance from start to last position: {distance:.2f} meters")

    # Vizualizacija putanje
    plt.figure(figsize=(8, 8))
    plt.plot(y_positions, z_positions, label="Put leta")

    # označi posebno početnu i završnu točku
    plt.scatter(y_positions[-1], z_positions[-1], color='red', label="Završna pozicija")
    plt.scatter(start_pos[1], start_pos[2], color='green', label="Početna pozicija", zorder=5)  # Početna pozicija

    plt.scatter(special_y, special_z, color='purple', label=f"Posebne točke", marker='x')
    plt.title("Vizualizacija dronovog leta")
    plt.xlabel("Y Position (m)")
    plt.ylabel("Z Position (m)")
    plt.legend()
    plt.grid(True)
    plt.show()


# kontroler koji omogućuje precizne pokrete
def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

# funkcija koja služi za estimaciju pozicije drona, dron tek kreće kada se pozicija estimira
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


# funkcija za resetiranje Kalmanovog filtra
def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def izvedi_trajektoriju():

    print("Usao u funkciju iz trajekt")

    cflib.crtp.init_drivers()

    print("spojen na drajvere")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    
        print("Uspia se sincat")

        # visina na kojoj ce dron startati
        cf = scf.cf

        with open("simplified_coords.pkl", "rb") as f:
            simplified_coords = pickle.load(f)
            print("Učitane spremljene koordinate:", simplified_coords)
            
        print("\nSimplicifirane koordinate", simplified_coords, "\n")
     
        simplified_coords = scale_points(simplified_coords, 2)
        simplified_coords = add_x_coordinate_to_points(simplified_coords)

        starting_point = simplified_coords[0]
        #print("\nAbsolute points 1: \n")
        simplified_coords = adjust_points_to_start_at_takeoff_point(simplified_coords, starting_point)
        simplified_coords = adjust_points_so_drone_is_above_ground(simplified_coords)
        print("\nSimplified points 2: \n")
        print(simplified_coords)
        totalLength = calculate_total_length_of_trajectory(simplified_coords)
    
        print("Total length of trajectory: \n", totalLength)

        # vrijeme potrebno da izvede putanju
        timeToExecute = 7

        points_with_time = add_time_and_yaw_to_points(simplified_coords, timeToExecute, totalLength)
        data = np.array(points_with_time)

        # ispiši točke koje su sada pripremljene za funkciju generate trajectory
        print("\nPoints ready for generating function: \n")
        print(data)

        # Generiranje trajektorije
        num_pieces = len(data) - 1
        print("\nGenerating trajectory, this migh take some time...\n")
        traj = generate_trajectory.generate_trajectory(data, num_pieces)

        # Spremanje trajektorije u csv formatu
        traj.savecsv("generated_trajectory.csv")

        file_path = "generated_trajectory.csv"
        trajektorija = np.loadtxt(file_path, delimiter=',', skiprows=1)

        # ispiši generirane polinome
        print("\nTrajectory generated: \n")
        print(trajektorija)

        # pozovi sve potrebne funkcije redom
        trajectory_id = 1
        duration = upload_trajectory(cf, trajectory_id, trajektorija, timeToExecute)
        print('\nThe sequence is {:.1f} seconds long\n'.format(duration))
        run_sequence(cf, trajectory_id, duration, points_with_time)


if __name__ == '__main__':
   
    # visina na kojoj će dron letjeti
    height = 0.3

    #Prikaz početnih gumba
    #prikazi_dodatne_gumbe()

    izvedi_trajektoriju()
