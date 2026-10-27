"""Detection HSV du plus grand rectangle de couleur dans l'image BGR."""

from numbers import Integral

import cv2
import numpy as np

from camera import Camera


class Detecteur_hsv:
    def __init__(self, camera=None):
        self.camera = Camera() if camera is None else camera

    def demarrer(self):
        self.camera.demarrer()

    def arreter(self):
        self.camera.arreter()

    def detecter(self, couleur_min, couleur_max):
        minimum = self._valider_couleur(couleur_min)
        maximum = self._valider_couleur(couleur_max)
        if any(bas > haut for bas, haut in zip(minimum, maximum)):
            raise ValueError("Chaque HSV minimum doit etre inferieur ou egal au maximum.")

        image = self.camera.capturer()
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_bin = cv2.inRange(
            image_hsv, np.array(minimum, dtype=np.uint8), np.array(maximum, dtype=np.uint8)
        )
        contours, _ = cv2.findContours(image_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        aire_max = 0
        p_haut_gauche = (0, 0)
        p_bas_droite = (0, 0)
        rect_centre = (0, 0)
        for contour in contours:
            x, y, largeur, hauteur = cv2.boundingRect(contour)
            aire = largeur * hauteur
            if aire > aire_max:
                aire_max = aire
                p_haut_gauche = (x, y)
                p_bas_droite = (x + largeur, y + hauteur)
                rect_centre = (x + largeur / 2, y + hauteur / 2)

        return image, image_bin, aire_max, p_haut_gauche, p_bas_droite, rect_centre

    @staticmethod
    def _valider_couleur(couleur):
        try:
            composantes = tuple(couleur)
        except TypeError as erreur:
            raise ValueError("Un HSV doit contenir trois entiers.") from erreur
        if len(composantes) != 3:
            raise ValueError("Un HSV doit contenir trois entiers.")
        for valeur, limite in zip(composantes, (179, 255, 255)):
            if (
                isinstance(valeur, bool)
                or not isinstance(valeur, Integral)
                or not 0 <= valeur <= limite
            ):
                raise ValueError("Les composantes HSV doivent respecter H=0..179, S/V=0..255.")
        return tuple(int(valeur) for valeur in composantes)
