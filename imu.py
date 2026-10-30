"""Navigation inertielle, avec les axes ay, gx et (my, mz) de l'enonce."""

import math
from collections import deque

from param import FENETRE_BIAIS, GRAVITE, SENS_CAP_MAGNETIQUE


class IMU:
    def __init__(self, imu):
        self.imu = imu
        self.accel_data = None
        self.magne_data = None
        self._ay_repos = deque(maxlen=FENETRE_BIAIS)
        self._gx_repos = deque(maxlen=FENETRE_BIAIS)
        self.ay_biais = 0.0
        self.gx_biais = 0.0
        self.corr_my = 0.0
        self.corr_mz = 0.0
        self.calibration_magnetometre = False
        self.min_my = self.min_mz = math.inf
        self.max_my = self.max_mz = -math.inf
        self.reinitialiser_deplacement()

    @property
    def biais_pret(self):
        return len(self._ay_repos) == FENETRE_BIAIS

    @property
    def vitesse_angulaire(self):
        if self.accel_data is None:
            return 0.0
        return self.accel_data[3] - self.gx_biais

    def reinitialiser_deplacement(self):
        self.vy = 0.0
        self.dist_y = 0.0
        self.angle_rot = 0.0
        self._ay_precedent = 0.0
        self._gx_precedent = 0.0

    def obtenir_deplacement(self, est_actif, dt, *, calibrer_biais=True):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt doit etre positif et fini")
        self.accel_data = tuple(self.imu.read_accelerometer_gyro_data())
        if len(self.accel_data) != 6 or not all(map(math.isfinite, self.accel_data)):
            raise ValueError("Lecture accelerometre/gyroscope invalide")
        _, ay, _, gx, _, _ = self.accel_data

        if not est_actif:
            if calibrer_biais:
                self._ay_repos.append(ay)
                self._gx_repos.append(gx)
                self.ay_biais = sum(self._ay_repos) / len(self._ay_repos)
                self.gx_biais = sum(self._gx_repos) / len(self._gx_repos)
            self.reinitialiser_deplacement()
        else:
            ay -= self.ay_biais
            gx -= self.gx_biais
            vy_precedent = self.vy
            self.vy += dt * (ay + self._ay_precedent) / 2 * GRAVITE
            self.dist_y += dt * (self.vy + vy_precedent) / 2
            self.angle_rot += dt * (gx + self._gx_precedent) / 2
            self._ay_precedent = ay
            self._gx_precedent = gx

        return self.angle_rot, self.dist_y

    def commencer_calibration_magnetometre(self):
        self.min_my = self.min_mz = math.inf
        self.max_my = self.max_mz = -math.inf
        self.calibration_magnetometre = True

    def terminer_calibration_magnetometre(self):
        if self.max_my <= self.min_my or self.max_mz <= self.min_mz:
            raise ValueError("Calibration magnetique incomplete : verifier les axes my/mz")
        self.calibration_magnetometre = False

    def obtenir_cap_magnetique(self):
        self.magne_data = tuple(self.imu.read_magnetometer_data())
        if len(self.magne_data) != 3 or not all(map(math.isfinite, self.magne_data)):
            raise ValueError("Lecture magnetometre invalide")
        _, my, mz = self.magne_data
        if self.calibration_magnetometre:
            self.min_my = min(self.min_my, my)
            self.max_my = max(self.max_my, my)
            self.min_mz = min(self.min_mz, mz)
            self.max_mz = max(self.max_mz, mz)
            self.corr_my = (self.max_my + self.min_my) / 2
            self.corr_mz = (self.max_mz + self.min_mz) / 2
        angle = math.atan2(mz - self.corr_mz, my - self.corr_my)
        return int(math.degrees(angle) * SENS_CAP_MAGNETIQUE % 360)
