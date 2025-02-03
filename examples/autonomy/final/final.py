import pickle
from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
from math import sqrt
import tkinter as tk
import os
import matplotlib.pyplot as plt
import re

simplified_coords = []

def rotate_coordinates_90_left(coords):
    print("before swap:",coords)
    newCoords = [(x, 1-y) for x, y in coords]
    print("after swap:",newCoords)
    return newCoords

def ramer_douglas_peucker(points, epsilon):
    if len(points) < 3:
        return points

    # Nađi najudaljeniju točku od pravca između prve i zadnje točke
    start, end = points[0], points[-1]
    max_dist = 0
    index = 0
    for i, point in enumerate(points[1:-1], start=1):
        x0, y0 = point
        x1, y1 = start
        x2, y2 = end
        dist = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if dist > max_dist:
            max_dist = dist
            index = i

    # Ako je maksimalna udaljenost veća od epsilon, rekurzivno podijeli
    if max_dist > epsilon:
        left = ramer_douglas_peucker(points[:index + 1], epsilon)
        right = ramer_douglas_peucker(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [start, end]

def detect_and_plot_object(images):

    global simplified_coords
   
    model = YOLO("yolov8n.pt")

    coordinates = []

    for img_path in images:
        img = cv2.imread(img_path)
        img_height, img_width, _ = img.shape
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        results = model(img)

        # traži čovjeka
        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = result
            if model.names[int(cls)] == "cell phone":
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                coordinates.append((center_x / img_width, center_y / img_height))
                break
    
    #crta nam putanju
    if coordinates:
        
        epsilon = 0.001 # Normalizirane koordinate zahtijevaju manji epsilon
        simplified_coords = ramer_douglas_peucker(coordinates, epsilon)
        
        while(len(simplified_coords) > 12):
            epsilon += 0.01
            simplified_coords = ramer_douglas_peucker(coordinates, epsilon)
        
        simplified_coords = rotate_coordinates_90_left(simplified_coords)
        simplified_x, simplified_y = zip(*simplified_coords)

        # Prikaz normaliziranih pojednostavljenih koordinata
        plt.figure(figsize=(10, 6))
        plt.scatter(simplified_x, simplified_y, color='green', label='Koordinate')
        plt.plot(simplified_x, simplified_y, linestyle='-', color='orange', alpha=0.8, label='Simplified Path')

        plt.title(f'Detektirane i filtrirane koordinate:')
        plt.xlabel('X koordinata (normalizirano)')
        plt.ylabel('Y koordinata (normalizirano)')
        plt.legend()
        plt.grid()
        plt.show()

        return simplified_coords 
    else:
        print("Nije pronađen niti jedan objekt na slikama.")
        return []

def natural_sort_key(s):
    # Razdvaja string na numeričke i nenumeričke dijelove
    return [int(text) if text.isdigit() else text.lower() for text in re.split('(\d+)', s)]

def load_images_from_directory(directory):
    """
    Učitava sve slike iz danog direktorija i vraća popis putanja do slika.
    Sortira slike prirodnim redoslijedom.
    """
    supported_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"}
    images = []

    for filename in sorted(os.listdir(directory), key=natural_sort_key):
        if os.path.splitext(filename)[1].lower() in supported_extensions:
            images.append(os.path.join(directory, filename))
    
    return images

def start_analysis():
    directory = "images"  # Putanja do direktorija sa slikama
    images = load_images_from_directory(directory)

    if not images:
        print(f"Nema dostupnih slika u direktoriju: {directory}")
        return

    simplified_coords = detect_and_plot_object(images)
    
    with open("simplified_coords.pkl", "wb") as f:
        pickle.dump(simplified_coords, f)
        print("Koordinate su spremljene u simplified_coords.pkl")
    print("Rezultat", f"Simplified Coordinates: {simplified_coords}")


def stop_program():
    global root
    root.destroy()

def main(): 
    global root
    root = tk.Tk()
    root.title("Putanja dobivena YOLO algoritmom") #u sta ovo promjeniti?

    frame = tk.Frame(root)
    frame.pack(pady=20, padx=20)

    start_button = tk.Button(frame, text="Start", command=start_analysis, width=15, height=2, bg="green", fg="black")
    start_button.pack(side="left", padx=10)

    stop_button = tk.Button(frame, text="Stop", command=stop_program, width=15, height=2, bg="red", fg="black")
    stop_button.pack(side="right", padx=10)

    root.mainloop()

if __name__ == "__main__":
    main()

