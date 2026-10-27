"""Acquisition BGR par camera du Raspberry Pi."""

from numbers import Integral
import platform

import cv2
import numpy as np


class Camera:
    def __init__(self, largeur=320, hauteur=240):
        for valeur in (largeur, hauteur):
            if isinstance(valeur, bool) or not isinstance(valeur, Integral) or valeur <= 0:
                raise ValueError("Les dimensions de la camera doivent etre des entiers positifs.")
        self.largeur = int(largeur)
        self.hauteur = int(hauteur)
        self._camera = None
        self._type_camera = None
        self._demarree = False

    def demarrer(self):
        if self._camera is not None:
            return

        try:
            systeme = platform.system()
            if systeme == "Windows":
                self._type_camera = "opencv"
                self._camera = cv2.VideoCapture(0)
                if not self._camera.isOpened():
                    raise RuntimeError("La webcam 0 est indisponible.")
                self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.largeur)
                self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.hauteur)
            elif systeme == "Linux":
                from picamera2 import Picamera2

                self._type_camera = "picamera2"
                self._camera = Picamera2()
                configuration = self._camera.create_video_configuration(
                    main={"format": "RGB888", "size": (self.largeur, self.hauteur)}
                )
                self._camera.configure(configuration)
                self._demarree = True
                self._camera.start()
            else:
                raise RuntimeError(f"Systeme non pris en charge : {systeme}.")
        except BaseException as erreur:
            self._fermer_apres_erreur()
            if isinstance(erreur, Exception):
                raise RuntimeError(f"Impossible de demarrer la camera : {erreur}") from erreur
            raise

    def capturer(self):
        if self._camera is None:
            raise RuntimeError("La camera doit etre demarree avant une capture.")

        try:
            if self._type_camera == "opencv":
                reussite, image = self._camera.read()
                if not reussite:
                    raise RuntimeError("La webcam n'a pas retourne d'image.")
            else:
                image = self._camera.capture_array("main")
            if (
                not isinstance(image, np.ndarray)
                or image.ndim != 3
                or image.shape[2] != 3
                or image.dtype != np.uint8
                or image.size == 0
            ):
                raise RuntimeError("La camera n'a pas retourne une image BGR valide.")
            if image.shape[:2] != (self.hauteur, self.largeur):
                image = cv2.resize(image, (self.largeur, self.hauteur))
            return image
        except BaseException as erreur:
            self._fermer_apres_erreur()
            if isinstance(erreur, Exception):
                raise RuntimeError(f"Impossible de capturer une image : {erreur}") from erreur
            raise

    def arreter(self):
        camera = self._camera
        type_camera = self._type_camera
        demarree = self._demarree
        self._camera = None
        self._type_camera = None
        self._demarree = False
        if camera is None:
            return
        if type_camera == "opencv":
            camera.release()
        else:
            try:
                if demarree:
                    camera.stop()
            finally:
                camera.close()

    def _fermer_apres_erreur(self):
        try:
            self.arreter()
        except Exception:
            pass
