#!/usr/bin/env python3
"""Genere une texture "chiffre" (style MNIST) pour le panneau digit_panel (TP04).

Chiffre noir epais sur fond blanc, centre, image carree => reconnaissable par un
petit CNN entraine sur MNIST. Quelques chiffres (0..3) sont fournis ; ce script
permet d'en (re)generer d'autres.

    ros2 run bootcamp_vision generate_digit.py            # 0,1,2,3
    ros2 run bootcamp_vision generate_digit.py --digit 7  # un chiffre precis
"""

import argparse
import os
import sys


def default_out_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(
        here, '..', 'models', 'digit_panel', 'materials', 'textures'))


def generate(digit, size, out_dir):
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("OpenCV requis : sudo apt install python3-opencv")

    img = np.full((size, size), 255, dtype=np.uint8)
    text = str(digit)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = size / 110.0          # ~ remplit la hauteur
    thickness = max(2, int(size / 40))
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    org = ((size - tw) // 2, (size + th) // 2)
    cv2.putText(img, text, org, font, scale, (0,), thickness, cv2.LINE_AA)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'digit_{digit}.png')
    cv2.imwrite(path, img)
    print(f'[generate_digit] ecrit {path}')


def main():
    p = argparse.ArgumentParser(description='Genere des textures de chiffres (MNIST-like).')
    p.add_argument('--digit', type=int, default=None, help='Chiffre (defaut : 0,1,2,3).')
    p.add_argument('--size', type=int, default=256, help='Taille image (px).')
    p.add_argument('--out', default=None, help='Dossier de sortie.')
    args = p.parse_args()
    out_dir = args.out or default_out_dir()
    digits = [args.digit] if args.digit is not None else [0, 1, 2, 3]
    for d in digits:
        generate(d, args.size, out_dir)


if __name__ == '__main__':
    main()
