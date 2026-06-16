#!/usr/bin/env python3
"""Genere les textures de label des paquets de cafe (coffee_pack_1/2/3).

Labels simplifies (marque + torrefaction) sur fonds de couleur distincts, pour la
classification (3 classes de cafe). Ecrit label.png dans chaque modele.

    ros2 run bootcamp_vision generate_label.py
"""

import os
import sys

# (dossier modele, couleur fond BGR, texte marque, texte torrefaction, couleur texte BGR)
PACKS = [
    ('coffee_pack_1', (205, 230, 245), 'ARABICA',  'LEGER',   (60, 40, 20)),
    ('coffee_pack_2', (60, 90, 140),   'ROBUSTA',  'CORSE',   (240, 240, 240)),
    ('coffee_pack_3', (35, 40, 55),    'MOKA',     'INTENSE', (210, 220, 235)),
]


def models_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'models'))


def generate(model, bg_bgr, brand, roast, fg_bgr, w=512, h=320):
    import cv2
    import numpy as np
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = bg_bgr
    # bandeau du haut (accent)
    cv2.rectangle(img, (0, 0), (w, 60), tuple(int(c * 0.7) for c in bg_bgr), -1)
    # grain de cafe stylise (ellipse + trait)
    cv2.ellipse(img, (w // 2, 150), (46, 64), 0, 0, 360, fg_bgr, 4)
    cv2.line(img, (w // 2, 90), (w // 2, 210), fg_bgr, 3)
    # textes
    cv2.putText(img, brand, (30, 270), cv2.FONT_HERSHEY_DUPLEX, 1.6, fg_bgr, 3, cv2.LINE_AA)
    cv2.putText(img, roast, (32, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.1, fg_bgr, 2, cv2.LINE_AA)

    out_dir = os.path.join(models_dir(), model, 'materials', 'textures')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'label.png')
    cv2.imwrite(path, img)
    print(f'[generate_label] ecrit {path}')


def main():
    try:
        import cv2  # noqa: F401
    except ImportError:
        sys.exit("OpenCV requis : sudo apt install python3-opencv")
    for model, bg, brand, roast, fg in PACKS:
        generate(model, bg, brand, roast, fg)


if __name__ == '__main__':
    main()
