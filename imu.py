"""Navigation inertielle, avec les axes ay, gx et (my, mz) de l'enonce."""

import math
from collections import deque

from param import (
    ECART_TYPE_ACCEL_REPOS_MAX, ECART_TYPE_GYRO_REPOS_MAX,
    FACTEUR_ANGLE, FACTEUR_DISTANCE, FENETRE_BIAIS, GRAVITE,
    SENS_CAP_MAGNETIQUE, VERIFIER_STABILITE_BIAIS, VITESSE_GYRO_REPOS_MAX,
)


class IMU:
    def __init__(self, imu, facteur_distance=FACTEUR_DISTANCE, facteur_angle=FACTEUR_ANGLE,
                 verifier_stabilite=VERIFIER_STABILITE_BIAIS):
        self.imu = imu
        if not all(math.isfinite(f) and f > 0 for f in (facteur_distance, facteur_angle)):
            raise ValueError("Les facteurs de mesure doivent etre positifs et finis")
        self.facteur_distance = facteur_distance
        self.facteur_angle = facteur_angle
        self.verifier_stabilite = verifier_stabilite
        self.accel_data = None
        self.magne_data = None
        self._ay_repos = deque(maxlen=FENETRE_BIAIS)
        self._gx_repos = deque(maxlen=FENETRE_BIAIS)
        self._mesures_repos = deque(maxlen=FENETRE_BIAIS)
        self._repos_stable = False
        self._biais_disponible = False
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
        return self._biais_disponible

    @property
    def repos_stable(self):
        return self._repos_stable

    @property
    def vitesse_angulaire(self):
        if self.accel_data is None:
            return 0.0
        return (self.accel_data[3] - self.gx_biais) * self.facteur_angle

    @property
    def acceleration_corrigee(self):
        if self.accel_data is None:
            return 0.0
        return (self.accel_data[1] - self.ay_biais) * GRAVITE

    def _oublier_repos(self):
        self._mesures_repos.clear()
        self._ay_repos.clear()
        self._gx_repos.clear()
        self._repos_stable = False
        self._biais_disponible = False

    def _mesurer_biais(self):
        self._mesures_repos.append(self.accel_data)
        self._ay_repos.append(self.accel_data[1])
        self._gx_repos.append(self.accel_data[3])
        self._repos_stable = False
        self._biais_disponible = False
        if len(self._mesures_repos) < FENETRE_BIAIS:
            return
        stable = True
        for axe, valeurs in enumerate(zip(*self._mesures_repos)):
            moyenne = sum(valeurs) / FENETRE_BIAIS
            variance = sum((v - moyenne) ** 2 for v in valeurs) / FENETRE_BIAIS
            limite = ECART_TYPE_ACCEL_REPOS_MAX if axe < 3 else ECART_TYPE_GYRO_REPOS_MAX
            if variance > limite ** 2:
                stable = False
                break
            if axe >= 3 and max(map(abs, valeurs)) > VITESSE_GYRO_REPOS_MAX:
                stable = False
                break
        self._repos_stable = stable
        if stable or not self.verifier_stabilite:
            self.ay_biais = sum(self._ay_repos) / FENETRE_BIAIS
            self.gx_biais = sum(self._gx_repos) / FENETRE_BIAIS
            self._biais_disponible = True

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
                self._mesurer_biais()
            else:
                self._oublier_repos()
            self.reinitialiser_deplacement()
        else:
            self._oublier_repos()
            ay -= self.ay_biais
            gx = (gx - self.gx_biais) * self.facteur_angle
            vy_precedent = self.vy
            self.vy += dt * (ay + self._ay_precedent) / 2 * GRAVITE
            self.dist_y += dt * (self.vy + vy_precedent) / 2
            self.angle_rot += dt * (gx + self._gx_precedent) / 2
            self._ay_precedent = ay
            self._gx_precedent = gx

        return self.angle_rot, self.dist_y * self.facteur_distance

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
        return self.cap_depuis_mesure(self.magne_data)

    def cap_depuis_mesure(self, mesure):
        _, my, mz = mesure
        angle = math.atan2(mz - self.corr_mz, my - self.corr_my)
        return int(math.degrees(angle) * SENS_CAP_MAGNETIQUE % 360)
