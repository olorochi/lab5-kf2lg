"""Rotation commander en angle."""

import math
from collections import deque
from orientation import CapDeroule, moyenne_caps, ecart_cap
from param import (TOLERANCE_ROTATION, IMPULSION_ROTATION_MAX, IMPULSION_ROTATION_MIN,
                   REPOS_ROTATION, TEMPS_STABILISATION, GAIN_ROTATION_FINE,
                   MSG_ARRETER, MSG_PIVOTER_G, MSG_PIVOTER_D)


class RotationControlee:
    def __init__(self, robot, imu, angle, maintenant, *, calibration=False,
                 sens_cap_gauche=-1, sens_gyro_gauche=None):
        self.robot, self.imu = robot, imu
        self.angle = float(angle)
        self.calibration = calibration
        self.sens_cap_gauche = sens_cap_gauche
        self.sens_gyro_gauche = sens_gyro_gauche
        self.gyro_depart = imu.angle_total
        self.mesure_depart = imu.magne_data
        self.cap_depart = imu.cap_precis
        self.progression = 0.0
        self.etat = "REPOS"
        self.commande = MSG_ARRETER
        self.fin_impulsion = None
        self.debut_repos = maintenant
        self.debut = maintenant
        self.vitesse = 90.0
        self._derniere_progression = 0.0
        self._duree_derniere = 0.0
        self._debut_moteur = maintenant
        self._caps = deque(maxlen=3)
        self._calibration_finie = not calibration
        self._mesure_repos_precedente = None
        if calibration:
            self.imu.commencer_calibration_magnetometre()

    @property
    def en_mouvement(self):
        return self.etat == "MARCHE"

    @property
    def termine(self):
        return self.etat == "TERMINE"

    @property
    def gyro_progression(self):
        delta = self.imu.angle_total-self.gyro_depart
        if self.sens_gyro_gauche is None:
            return abs(delta)
        return delta*self.sens_gyro_gauche

    def arreter_moteurs(self, maintenant):
        self.robot.arreter()
        self.commande = MSG_ARRETER
        self.etat = "REPOS"
        self.fin_impulsion = None
        self.debut_repos = maintenant
        self._duree_derniere = maintenant-self._debut_moteur
        self._caps.clear()

    def annuler(self, maintenant):
        self.arreter_moteurs(maintenant)
        self.etat = "ANNULEE"

    def _tourner(self, maintenant, restant):
        if restant >= 0:
            self.robot.pivoter_gauche()
            self.commande = MSG_PIVOTER_G
        else:
            self.robot.pivoter_droite()
            self.commande = MSG_PIVOTER_D
        self.etat = "MARCHE"
        self._debut_moteur = maintenant
        if self.calibration or abs(restant) < 25:
            duree = GAIN_ROTATION_FINE*abs(restant)/self.vitesse
            duree = max(IMPULSION_ROTATION_MIN, min(IMPULSION_ROTATION_MAX, duree))
            self.fin_impulsion = maintenant+duree
        else:
            self.fin_impulsion = None

    def actualiser(self, maintenant, nouveau_cap=False):
        if self.etat in ("TERMINE", "ANNULEE"):
            return
        if self.en_mouvement:
            delta_gyro = self.imu.angle_total-self.gyro_depart
            if self.sens_gyro_gauche is None and abs(delta_gyro) > 5:
                self.sens_gyro_gauche = 1 if delta_gyro > 0 else -1
            rate = abs(self.imu.vitesse_angulaire)
            if rate > 5:
                self.vitesse = 0.8*self.vitesse+0.2*rate
            freinage = max(5.0, self.vitesse*0.10)
            cible_proche = (not self.calibration and self.fin_impulsion is None
                            and self.gyro_progression >= self.angle-freinage)
            if (self.fin_impulsion is not None and maintenant >= self.fin_impulsion) or cible_proche:
                self.arreter_moteurs(maintenant)
            return
        if maintenant-self.debut_repos < TEMPS_STABILISATION:
            return
        if nouveau_cap:
            self._caps.append(self.imu.cap_precis)
        if maintenant-self.debut_repos < REPOS_ROTATION or len(self._caps) < 3:
            return
        if self.calibration and not self._calibration_finie:
            if self.gyro_progression >= 720:
                self.imu.terminer_calibration_magnetometre()
                self.cap_depart = self.imu.cap_depuis_mesure(self.mesure_depart)
                if self._mesure_repos_precedente is not None:
                    avant = self.imu.cap_depuis_mesure(self._mesure_repos_precedente)
                    delta = ecart_cap(self.imu.cap_precis, avant)
                    if abs(delta) > 1:
                        self.sens_cap_gauche = 1 if delta > 0 else -1
                self._caps.clear()
                self._calibration_finie = True
                return
            self._mesure_repos_precedente = self.imu.magne_data
            self._tourner(maintenant, self.angle-self.gyro_progression)
            return
        cap = moyenne_caps(self._caps)
        approche = self.sens_cap_gauche*self.gyro_progression
        suivi = CapDeroule.depuis_reference(self.cap_depart, cap, approche)
        self.progression = self.sens_cap_gauche*suivi.progression
        if self._duree_derniere > 0.001:
            mouvement = abs(self.progression-self._derniere_progression)
            if mouvement > 0.5:
                self.vitesse = max(10.0, min(360.0, mouvement/self._duree_derniere))
        self._derniere_progression = self.progression
        restant = self.angle-self.progression
        print(f"Rotation : {self.progression:.1f}/{self.angle:.1f} degres; "
              f"cap {cap:.1f}; depart {self.cap_depart:.1f}.")
        if abs(restant) <= TOLERANCE_ROTATION:
            self.robot.arreter()
            self.etat = "TERMINE"
        else:
            self._tourner(maintenant, restant)
