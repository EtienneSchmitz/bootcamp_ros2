#!/usr/bin/env python3
"""Genere une (ou plusieurs) image de marqueur ArUco pour le colis aruco_parcel.

Dictionnaire DICT_4X4_50 (ID = classe utilisee par le detecteur). Bord blanc
(quiet zone) inclus pour la detection. Quelques IDs (0,1,2) sont fournis ; ce
script permet d'en (re)generer d'autres.

    ros2 run bootcamp_vision generate_aruco.py            # IDs 0,1,2
    ros2 run bootcamp_vision generate_aruco.py --id 7     # un ID precis
"""

import argparse
import os
import sys


def default_out_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(
        here, '..', 'models', 'aruco_parcel', 'materials', 'textures'))


def generate(marker_id, size, out_dir):
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("OpenCV requis : sudo apt install python3-opencv")
    if not hasattr(cv2, 'aruco'):
        sys.exit("cv2.aruco absent : pip install opencv-contrib-python")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    else:
        marker = cv2.aruco.drawMarker(aruco_dict, marker_id, size)

    pad = int(size * 0.12)
    canvas = np.full((size + 2 * pad, size + 2 * pad), 255, dtype=np.uint8)
    canvas[pad:pad + size, pad:pad + size] = marker

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'aruco_{marker_id}.png')
    cv2.imwrite(path, canvas)
    print(f'[generate_aruco] ecrit {path}')


def main():
    p = argparse.ArgumentParser(description='Genere des textures de marqueurs ArUco.')
    p.add_argument('--id', type=int, default=None, help='ID (defaut : 0,1,2).')
    p.add_argument('--size', type=int, default=512, help='Taille marqueur (px).')
    p.add_argument('--out', default=None, help='Dossier de sortie.')
    args = p.parse_args()
    out_dir = args.out or default_out_dir()
    ids = [args.id] if args.id is not None else [0, 1, 2]
    for marker_id in ids:
        generate(marker_id, args.size, out_dir)


if __name__ == '__main__':
    main()
