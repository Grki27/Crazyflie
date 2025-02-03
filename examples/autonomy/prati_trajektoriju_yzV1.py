import sys
import time
import logging

import numpy as np
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

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E7E7')

# samo ispisi greske
logging.basicConfig(level=logging.ERROR)

# side_length = 0.8  # Duljina stranice četverokuta

# Definicija točaka (x, y, z)
#absolute_points = [
#    (0, 0),  # Početna točka
#    (side_length, 0),  # Desna točka
#    (side_length, side_length),  # Gornja desna točka
#    (0, side_length),  # Gornja lijeva točka
#    (0, 0)  # Povratak na početnu točku
#]

radius = 0.6  # Radijus kružnice (u metrima)
num_points = 8

# neke random tocke, u ovom slucaju za kruznicu, koje dron mora odletiti
absolute_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points  # Kut u radijanima
    x = radius * math.cos(angle)  # X koordinata
    y = radius * math.sin(angle)  # Y koordinata
    absolute_points.append((x, y))


# ovdje kreće nas dio, tj. prilagođavanje početnih točaka + izvođenje putanje

# dodajemo koordinatu visine putanji
def add_z_coordinate_to_points(points, height):
    return [(x, y, height) for x, y in points]


# prilagođavamo dobivene koordinate tako da prva počinje u ishodištu koordinatnog sustava
def adjust_points_to_start_at_takeoff_point(points, starting_point):
    x_start, y_start, z_start = starting_point
    return [(x - x_start, y - y_start, z) for x, y, z in points]
    
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

    # Dodaj udaljenost povratka na početnu točku 
    last_x, last_y, last_z = starting_point
    x_prev, y_prev, z_prev = points[len(points) - 1]
    total_length_of_trajectory += math.sqrt((last_x - x_prev)**2 + (last_y - y_prev)**2 + (last_z - z_prev)**2)

    return total_length_of_trajectory

# funkcija koja dodaje vrijeme u kojem se dron mora naci u nekoj tocki, kao i yaw (koji je 0)
# ovo je zapravo svojevrsna priprema podataka prije nego što ih po+aljemo u generate_trajectory
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


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

# kontroler koji omogućuje precizne pokrete
def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

# funkcija koja uploada svaki redak pripremljene trajektorije u crazyflieovu memoriju, vraća ukupnu duljinu trajektorije
def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

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
def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(0.3, 2.0)
    time.sleep(3.0)
    relative = False
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
   
    cflib.crtp.init_drivers()

    # logiranje pozicije

    lg_position = LogConfig(name='Position', period_in_ms=100) # logiraj svaku sekundu x, y i z
    lg_position.add_variable('kalman.stateX', 'float')  # x pozicija
    lg_position.add_variable('kalman.stateY', 'float')  # y pozicija
    lg_position.add_variable('kalman.stateZ', 'float')  # z pozicija

    # visina na kojoj ce dron letjeti
    height = 0.3

    # vrijeme potrebno da izvede putanju
    timeToExecute = 7

    absolute_points = add_z_coordinate_to_points(absolute_points, height)
    starting_point = absolute_points[0]
    absolute_points = adjust_points_to_start_at_takeoff_point(absolute_points, starting_point)
    starting_point = absolute_points[0]
    totalLength = calculate_total_length_of_trajectory(absolute_points)
    points_with_time = add_time_and_yaw_to_points(absolute_points, timeToExecute, totalLength)
    data = np.array(points_with_time)

    # ispiši točke koje su sada pripremljene za funkciju generate trajectory
    print(data)

    # Generiranje trajektorije
    num_pieces = len(points_with_time) - 1
    print("Generating trajectory, this migh take some time...")
    traj = generate_trajectory.generate_trajectory(data, num_pieces)

    # Spremanje trajektorije u csv formatu
    traj.savecsv("generated_trajectory.csv")

    file_path = r'C:\Users\izakarij\Desktop\crazyflie-lib-python-master\generated_trajectory.csv'
    trajektorija = np.loadtxt(file_path, delimiter=',', skiprows=1)

    # ispiši generirane polinome
    print("Trajectory generated: ")
    print(trajektorija)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        # pozovi sve potrebne funkcije redom
        trajectory_id = 1
        duration = upload_trajectory(cf, trajectory_id, trajektorija)
        print('The sequence is {:.1f} seconds long'.format(duration))
        reset_estimator(cf)
        run_sequence(cf, trajectory_id, duration)
