"""IMU : biais au repos, integration trapezoidale et compensation d'inclinaison."""

import math
from collections import deque
from param import (FENETRE_BIAIS, GRAVITE, SENS_CAP_MAGNETIQUE, ICM20948_DT,
                   FILTRE_ACCEL_TAU, FACTEUR_DISTANCE, FACTEUR_ANGLE)


def tourner_vecteur(vecteur, rotation):
    """Rotation de Rodrigues; rotation est un vecteur d'angles en radians."""
    angle = math.sqrt(sum(x*x for x in rotation))
    if angle < 1e-12:
        return list(vecteur)
    x, y, z = (r / angle for r in rotation)
    vx, vy, vz = vecteur
    c, s = math.cos(angle), math.sin(angle)
    produit = x*vx + y*vy + z*vz
    croix = (y*vz-z*vy, z*vx-x*vz, x*vy-y*vx)
    return [v*c + k*s + u*produit*(1-c)
            for v, k, u in zip(vecteur, croix, (x,y,z))]


class IMU:
    def __init__(self, imu, filtre_tau=FILTRE_ACCEL_TAU):
        self.imu = imu
        if not math.isfinite(filtre_tau) or filtre_tau < 0:
            raise ValueError("La constante de filtrage doit etre positive ou nulle")
        if not all(math.isfinite(v) and v > 0 for v in (FACTEUR_DISTANCE, FACTEUR_ANGLE)):
            raise ValueError("Les facteurs de mesure doivent etre positifs et finis")
        self.filtre_tau = filtre_tau
        self.accel_data = None
        self.magne_data = None
        self._repos = deque(maxlen=FENETRE_BIAIS)
        self._somme_repos = [0.0]*6
        self._temps_repos = 0.0
        self.accel_repos = [1.0, 0.0, 0.0]
        self.gyro_biais = [0.0]*3
        self.ay_biais = self.gx_biais = 0.0
        self._gravite_initiale = [1.0, 0.0, 0.0]
        self.angle_total = 0.0
        self._gyro_total_precedent = 0.0
        self.acceleration_corrigee = 0.0
        self.vitesse_angulaire = 0.0
        self.corr_my = self.corr_mz = 0.0
        self.calibration_magnetometre = False
        self.min_my = self.min_mz = math.inf
        self.max_my = self.max_mz = -math.inf
        self.reinitialiser_deplacement()

    @property
    def biais_pret(self):
        return len(self._repos) == FENETRE_BIAIS

    def vider_fenetre_biais(self):
        self._repos.clear()
        self._somme_repos = [0.0]*6
        self._temps_repos = 0.0

    def reinitialiser_deplacement(self):
        self.vy = self.dist_y = self.angle_rot = 0.0
        self._ay_precedent = self._gx_precedent = self._a_filtree = 0.0
        self._gyro_precedent = [0.0]*3
        self._gravite = list(self._gravite_initiale)

    def _biais(self, dt):
        self._temps_repos += dt
        for i, v in enumerate(self.accel_data):
            self._somme_repos[i] += v*dt
        if self._temps_repos + 1e-9 < ICM20948_DT:
            return
        self._repos.append(tuple(v/self._temps_repos for v in self._somme_repos))
        self._somme_repos = [0.0]*6
        self._temps_repos = 0.0
        moyennes = [sum(l[i] for l in self._repos)/len(self._repos) for i in range(6)]
        self.accel_repos = moyennes[:3]
        self.gyro_biais = moyennes[3:]
        self.ay_biais, self.gx_biais = moyennes[1], moyennes[3]
        norme = math.sqrt(sum(v*v for v in self.accel_repos))
        if norme > 0.1:
            self._gravite_initiale = [v/norme for v in self.accel_repos]

    def obtenir_deplacement(self, est_actif, dt, *, calibrer_biais=True):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt doit etre positif et fini")
        self.accel_data = tuple(self.imu.read_accelerometer_gyro_data())
        if len(self.accel_data) != 6 or not all(map(math.isfinite, self.accel_data)):
            raise ValueError("Lecture accelerometre/gyroscope invalide")
        if not est_actif and calibrer_biais:
            self._biais(dt)
        gyro = [(v-b)*FACTEUR_ANGLE for v,b in zip(self.accel_data[3:], self.gyro_biais)]
        self.vitesse_angulaire = gyro[0]
        self.angle_total += dt*(gyro[0]+self._gyro_total_precedent)/2
        self._gyro_total_precedent = gyro[0]
        if not est_actif:
            self.reinitialiser_deplacement()
            self.acceleration_corrigee = 0.0
        else:
            self.vider_fenetre_biais()
            rotation = [-math.radians((g+p)/2)*dt
                        for g,p in zip(gyro, self._gyro_precedent)]
            self._gravite = tourner_vecteur(self._gravite, rotation)
            variation_gravite = self._gravite[1] - self._gravite_initiale[1]
            self.acceleration_corrigee = (self.accel_data[1]-self.ay_biais-variation_gravite)*GRAVITE
            alpha = dt/(self.filtre_tau+dt) if self.filtre_tau > 0 else 1.0
            self._a_filtree += alpha*(self.acceleration_corrigee-self._a_filtree)
            precedent = self.vy
            self.vy += dt*(self._a_filtree+self._ay_precedent)/2
            self.dist_y += dt*(self.vy+precedent)/2
            self.angle_rot += dt*(gyro[0]+self._gx_precedent)/2
            self._ay_precedent, self._gx_precedent = self._a_filtree, gyro[0]
            self._gyro_precedent = gyro
        return self.angle_rot, self.dist_y*FACTEUR_DISTANCE

    def commencer_calibration_magnetometre(self):
        self.min_my = self.min_mz = math.inf
        self.max_my = self.max_mz = -math.inf
        self.calibration_magnetometre = True

    def terminer_calibration_magnetometre(self):
        if self.max_my <= self.min_my or self.max_mz <= self.min_mz:
            raise ValueError("Calibration magnetique incomplete : verifier les axes my/mz")
        self.calibration_magnetometre = False

    def cap_depuis_mesure(self, mesure):
        _, my, mz = mesure
        return math.degrees(math.atan2(mz-self.corr_mz, my-self.corr_my))*SENS_CAP_MAGNETIQUE % 360

    @property
    def cap_precis(self):
        return self.cap_depuis_mesure(self.magne_data) if self.magne_data is not None else 0.0

    def obtenir_cap_magnetique(self):
        self.magne_data = tuple(self.imu.read_magnetometer_data())
        if len(self.magne_data) != 3 or not all(map(math.isfinite, self.magne_data)):
            raise ValueError("Lecture magnetometre invalide")
        if self.calibration_magnetometre:
            _, my, mz = self.magne_data
            self.min_my, self.max_my = min(self.min_my,my), max(self.max_my,my)
            self.min_mz, self.max_mz = min(self.min_mz,mz), max(self.max_mz,mz)
            self.corr_my = (self.max_my+self.min_my)/2
            self.corr_mz = (self.max_mz+self.min_mz)/2
        return int(self.cap_precis)
