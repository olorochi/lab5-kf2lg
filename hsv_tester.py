#!/usr/bin/env python3

"""Calibration HSV sur la camera du Pi, ou une webcam sous Windows."""

import cv2

from detecteur_hsv import Detecteur_hsv
from param import TEINTE_MAX, TEINTE_MIN

FENETRE = "Calibration HSV"
FENETRE_MASQUE = "Masque HSV"
COMPOSANTES = ("H", "S", "V")


def main():
    detecteur = Detecteur_hsv()
    couleur_min, couleur_max = TEINTE_MIN, TEINTE_MAX
    try:
        cv2.namedWindow(FENETRE)
        for index, composante in enumerate(COMPOSANTES):
            limite = 179 if index == 0 else 255
            cv2.createTrackbar(f"{composante} min", FENETRE,
                               couleur_min[index], limite, lambda _: None)
            cv2.createTrackbar(f"{composante} max", FENETRE,
                               couleur_max[index], limite, lambda _: None)
        detecteur.demarrer()
        while True:
            minima = []
            maxima = []
            for composante in COMPOSANTES:
                valeur_min = cv2.getTrackbarPos(f"{composante} min", FENETRE)
                valeur_max = cv2.getTrackbarPos(f"{composante} max", FENETRE)
                if valeur_min > valeur_max:
                    valeur_max = valeur_min
                    cv2.setTrackbarPos(f"{composante} max", FENETRE, valeur_max)
                minima.append(valeur_min)
                maxima.append(valeur_max)
            couleur_min, couleur_max = tuple(minima), tuple(maxima)
            image, masque, aire, p_hg, p_bd, _ = detecteur.detecter(couleur_min, couleur_max)
            if aire:
                cv2.rectangle(image, p_hg, p_bd, (0, 255, 0), 2)
            cv2.putText(image, f"Aire rectangle: {aire} px", (5, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow(FENETRE, image)
            cv2.imshow(FENETRE_MASQUE, masque)
            touche = cv2.waitKeyEx(30)
            if touche in (ord("q"), ord("x"), 27):
                break
            if cv2.getWindowProperty(FENETRE, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        detecteur.arreter()
        cv2.destroyAllWindows()
        print(f"TEINTE_MIN = {couleur_min}\nTEINTE_MAX = {couleur_max}")


if __name__ == "__main__":
    main()
