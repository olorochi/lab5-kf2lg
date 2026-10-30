#!/bin/python3

"""Controleur du robot."""

import argparse
import csv
import math
import time
from contextlib import ExitStack
from collections import deque

from ev_app import EvApp
from ev_app_client_api import gen_ev_externe
from imu import IMU
from moteur import Moteur
from param import (
    APP_CTRL_ROBOT,
    APP_PATROUILLEUR,
    DELAI_CALIBRATION_MAX,
    DELAI_ALIGNEMENT_MAX,
    DELAI_COMMUNICATION,
    DUREE_ARRET,
    GAIN_CORRECTION_CAP,
    IMPULSION_CALIBRATION,
    IMPULSION_CORRECTION_MAX,
    IMPULSION_CORRECTION_MIN,
    INTERVALLE_CONTROLE,
    INTERVALLE_IMU_MAX,
    IP_PATROUILLEUR,
    MOTEUR_DROIT_IN1,
    MOTEUR_DROIT_IN2,
    MOTEUR_DROIT_PWM,
    MOTEUR_GAUCHE_IN1,
    MOTEUR_GAUCHE_IN2,
    MOTEUR_GAUCHE_PWM,
    MSG_ARRETER,
    MSG_AVANCER,
    MSG_IMU_INFO,
    MSG_PIVOTER_D,
    MSG_PIVOTER_G,
    MSG_RECULER,
    MSG_VITESSE,
    NB_TOURS_CALIBRATION,
    NB_CORRECTIONS_CAP_MAX,
    REPOS_CALIBRATION,
    TEMPS_STABILISATION,
    TOLERANCE_ALIGNEMENT_INITIAL,
    VITESSE_ANGULAIRE_INITIALE,
    VITESSE_MAX,
    VITESSE_MIN,
)
from robot import Robot


class CtrlRobot(EvApp):
    def __init__(self, port_no, robot, imu, horloge=time.monotonic,
                 envoyer=gen_ev_externe, calibrer=True, journal=None, **app_options):
        app_options.setdefault("tmo", INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.robot = robot
        self.imu = imu
        self.horloge = horloge
        self.envoyer = envoyer
        self._exiger_biais = calibrer
        self._attente_biais = False
        self._journal = journal
        self._csv = csv.writer(journal) if journal is not None else None
        if self._csv is not None:
            self._csv.writerow(("temps_s", "dt_s", "phase", "commande", "ax_g", "ay_g", "az_g", "biais_ay_g",
                                "ay_corr_m_s2", "gx_deg_s", "gy_deg_s", "gz_deg_s", "biais_gx_deg_s",
                                "vy_m_s", "distance_brute_m", "distance_m",
                                "angle_deg", "cap_deg", "biais_stable", "biais_pret"))
        maintenant = horloge()
        self._dernier_echantillon = maintenant
        self._prochain_echantillon = maintenant + INTERVALLE_CONTROLE
        self._derniere_commande = maintenant
        self._biais_apres = maintenant
        self._phase = "BIAIS" if calibrer else "PRET"
        self._debut_phase = maintenant
        self._angle_calibration = 0.0
        self._gx_calibration_precedent = 0.0
        self._calibration_en_rotation = False
        self._fin_impulsion_calibration = None
        self._reprise_calibration = maintenant
        self._mag_depart = None
        self._cap_depart = None
        self._caps_alignement = deque(maxlen=5)
        self._corrections_alignement = 0
        self.action = MSG_ARRETER
        self._actions = {
            MSG_ARRETER: self.robot.arreter,
            MSG_AVANCER: self.robot.avancer,
            MSG_RECULER: self.robot.reculer,
            MSG_PIVOTER_G: self.robot.pivoter_gauche,
            MSG_PIVOTER_D: self.robot.pivoter_droite,
        }
        self.robot.arreter()
        if calibrer:
            print("Calibration : laisser le robot immobile pour mesurer le biais sur 20 lectures.")

    @staticmethod
    def lire_vitesse(evenement):
        donnees = evenement.split()
        if len(donnees) != 1 or not donnees[0]:
            raise ValueError("Une seule valeur de vitesse est requise")
        vitesse = float(donnees[0])
        if not math.isfinite(vitesse):
            raise ValueError("La vitesse doit etre finie")
        return max(VITESSE_MIN, min(VITESSE_MAX, vitesse))

    def _arreter(self):
        self.robot.arreter()
        if self.action != MSG_ARRETER or self._phase == "MAGNETOMETRE":
            self._biais_apres = self.horloge() + TEMPS_STABILISATION
        self.action = MSG_ARRETER

    def _appliquer_mouvement(self, commande):
        changement = commande != self.action
        if (commande != MSG_ARRETER and self.action == MSG_ARRETER
                and self._exiger_biais and not self.imu.biais_pret):
            if not self._attente_biais:
                print("Attente de 20 mesures de repos pour le biais; laisser le robot immobile.")
            self._attente_biais = True
            return
        self._attente_biais = False
        if commande == MSG_ARRETER:
            self._arreter()
        else:
            self._actions[commande]()
            self.action = commande
        if changement:
            print(f"Commande: {commande}, PWM: {self.robot.vitesse:.2f}.")

    def _traiter_commande(self, evenement):
        if evenement.type == MSG_VITESSE:
            self.robot.vitesse = self.lire_vitesse(evenement)
            self._actions[self.action]()
            print(f"Vitesse PWM: {self.robot.vitesse:.2f}.")
            return

        if evenement.type in self._actions:
            self._appliquer_mouvement(evenement.type)

    def dispatch_event(self, evenement):
        if evenement is None:
            return
        try:
            if self._phase != "PRET":
                if evenement.type == MSG_ARRETER:
                    self._arreter()
                    self.quitter_app()
                    print("Calibration annulee par une commande d'arret.")
                return
            self._traiter_commande(evenement)
            if evenement.type in self._actions:
                self._derniere_commande = self.horloge()
        except (TypeError, ValueError) as erreur:
            self._arreter()
            print(f"Commande invalide, robot arrete: {erreur}")

    def _commencer_impulsion_calibration(self, maintenant):
        restant = NB_TOURS_CALIBRATION * 360 - abs(self._angle_calibration)
        vitesse = max(30.0, abs(self.imu.vitesse_angulaire))
        duree = min(IMPULSION_CALIBRATION, max(0.01, restant / vitesse))
        self.robot.pivoter_gauche()
        self._calibration_en_rotation = True
        self._fin_impulsion_calibration = maintenant + duree

    def _arreter_impulsion_calibration(self, maintenant):
        self.robot.arreter()
        self._calibration_en_rotation = False
        self._fin_impulsion_calibration = None
        self._biais_apres = maintenant + TEMPS_STABILISATION
        self._reprise_calibration = maintenant + REPOS_CALIBRATION
        self._caps_alignement.clear()

    def _terminer_alignement(self, maintenant, message):
        self._arreter_impulsion_calibration(maintenant)
        self._phase = "REPOS"
        self._debut_phase = maintenant
        print(message)

    def _calibrer(self, maintenant, dt, cap_mag):
        if self._phase == "BIAIS" and self.imu.biais_pret:
            self._mag_depart = self.imu.magne_data
            self.imu.commencer_calibration_magnetometre()
            self._phase = "MAGNETOMETRE"
            self._debut_phase = maintenant
            self._commencer_impulsion_calibration(maintenant)
            print(f"Calibration magnetique : {NB_TOURS_CALIBRATION} tours par impulsions; "
                  "mesures magnetiques prises moteurs arretes.")
        elif self._phase == "MAGNETOMETRE":
            if maintenant - self._debut_phase > DELAI_CALIBRATION_MAX:
                raise RuntimeError("Calibration trop longue : verifier le gyroscope et les moteurs")
            gx = self.imu.vitesse_angulaire
            self._angle_calibration += dt * (gx + self._gx_calibration_precedent) / 2
            self._gx_calibration_precedent = gx
            if self._calibration_en_rotation:
                if abs(self._angle_calibration) >= NB_TOURS_CALIBRATION * 360:
                    self._arreter_impulsion_calibration(maintenant)
            elif maintenant >= self._reprise_calibration:
                if abs(self._angle_calibration) >= NB_TOURS_CALIBRATION * 360:
                    self.imu.terminer_calibration_magnetometre()
                    self._cap_depart = self.imu.cap_depuis_mesure(self._mag_depart)
                    self._phase = "ALIGNEMENT"
                    self._debut_phase = maintenant
                    self._caps_alignement.clear()
                else:
                    self._commencer_impulsion_calibration(maintenant)
        elif self._phase == "ALIGNEMENT":
            if maintenant - self._debut_phase >= DELAI_ALIGNEMENT_MAX:
                self._terminer_alignement(
                    maintenant, "Ajustement initial termine : delai atteint. "
                    "Calibration magnetique conservee; reprise du biais au repos.")
                return
            if not self._calibration_en_rotation and maintenant >= self._biais_apres:
                self._caps_alignement.append(cap_mag)
                if (maintenant >= self._reprise_calibration
                        and len(self._caps_alignement) == self._caps_alignement.maxlen):
                    sinus = sum(math.sin(math.radians(c)) for c in self._caps_alignement)
                    cosinus = sum(math.cos(math.radians(c)) for c in self._caps_alignement)
                    cap_repos = math.degrees(math.atan2(sinus, cosinus)) % 360
                    erreur = (self._cap_depart - cap_repos + 180) % 360 - 180
                    if abs(erreur) <= TOLERANCE_ALIGNEMENT_INITIAL:
                        self._terminer_alignement(
                            maintenant, f"Cap initial retrouve : ecart {erreur:+.1f} degres.")
                    elif self._corrections_alignement >= NB_CORRECTIONS_CAP_MAX:
                        self._terminer_alignement(
                            maintenant, f"Ajustement initial termine apres "
                            f"{self._corrections_alignement} corrections : ecart residuel "
                            f"{erreur:+.1f} degres. Calibration magnetique conservee.")
                    else:
                        duree = GAIN_CORRECTION_CAP * abs(erreur) / VITESSE_ANGULAIRE_INITIALE
                        duree = max(IMPULSION_CORRECTION_MIN, min(IMPULSION_CORRECTION_MAX, duree))
                        mouvement = self.robot.pivoter_droite if erreur > 0 else self.robot.pivoter_gauche
                        mouvement()
                        self._calibration_en_rotation = True
                        self._fin_impulsion_calibration = maintenant + duree
                        self._corrections_alignement += 1
        elif (self._phase == "REPOS" and maintenant - self._debut_phase >= DUREE_ARRET
              and self.imu.biais_pret):
            self._phase = "PRET"
            print(f"Calibration terminee, controleur PRET : biais ay={self.imu.ay_biais:.6f} g, "
                  f"gx={self.imu.gx_biais:.4f} deg/s; "
                  f"corrections my={self.imu.corr_my:.3f}, mz={self.imu.corr_mz:.3f}.")

    def traiter_cycle(self):
        maintenant = self.horloge()
        etait_en_rotation = self._calibration_en_rotation
        if (self._fin_impulsion_calibration is not None
                and maintenant >= self._fin_impulsion_calibration):
            self._arreter_impulsion_calibration(maintenant)
        if (self._phase == "PRET" and self.action != MSG_ARRETER
                and maintenant - self._derniere_commande > DELAI_COMMUNICATION):
            self._arreter()
            raise RuntimeError("Patrouilleur sans nouvelles : robot arrete")
        if maintenant >= self._prochain_echantillon:
            dt = maintenant - self._dernier_echantillon
            if dt > INTERVALLE_IMU_MAX:
                en_mouvement = self.action != MSG_ARRETER or etait_en_rotation
                if en_mouvement:
                    self._arreter()
                    raise RuntimeError(
                        f"Echantillonnage IMU interrompu en mouvement (dt={dt:.3f} s) : robot arrete")
                print(f"Lecture retardee au repos (dt={dt:.3f} s); mesure du biais reprise.")
            self._dernier_echantillon = maintenant
            self._prochain_echantillon = maintenant + INTERVALLE_CONTROLE
            actif = self._phase == "PRET" and self.action != MSG_ARRETER
            angle_rot, dist_y = self.imu.obtenir_deplacement(
                actif, dt, calibrer_biais=(self._phase not in ("MAGNETOMETRE", "ALIGNEMENT")
                                         and maintenant >= self._biais_apres),
            )
            if self._phase == "MAGNETOMETRE":
                self.imu.calibration_magnetometre = (
                    not self._calibration_en_rotation and maintenant >= self._biais_apres)
            cap_mag = self.imu.obtenir_cap_magnetique()
            if self._phase != "PRET":
                self._calibrer(maintenant, dt, cap_mag)
            if self._phase == "PRET":
                self.envoyer(IP_PATROUILLEUR, APP_PATROUILLEUR, MSG_IMU_INFO,
                             angle_rot, dist_y, cap_mag)
            if self._csv is not None:
                self._csv.writerow((maintenant, dt, self._phase, self.action,
                                    *self.imu.accel_data[:3], self.imu.ay_biais,
                                    self.imu.acceleration_corrigee, self.imu.accel_data[3],
                                    *self.imu.accel_data[4:],
                                    self.imu.gx_biais, self.imu.vy, self.imu.dist_y,
                                    dist_y, angle_rot, cap_mag, self.imu.repos_stable,
                                    self.imu.biais_pret))
                self._journal.flush()
        self.tmo = max(0.0, self._prochain_echantillon - self.horloge())
        if self._fin_impulsion_calibration is not None:
            self.tmo = min(self.tmo, max(0.0, self._fin_impulsion_calibration - self.horloge()))

    def quitter(self):
        self.action = MSG_ARRETER
        try:
            self.robot.fermer()
        finally:
            if hasattr(self, "bus_i2c"):
                self.bus_i2c.close()
        print("Controleur arrete; moteurs desactives.")


def creer_controleur(port_no=APP_CTRL_ROBOT, journal=None):
    try:
        from gpiozero import DigitalOutputDevice, PWMOutputDevice
        from icm20948 import ICM20948
        from smbus2 import SMBus
    except ImportError as erreur:
        raise SystemExit("Installer gpiozero, icm20948 et smbus2 sur le Raspberry Pi") from erreur

    peripheriques = []

    def creer_sortie(classe, broche):
        sortie = classe(broche)
        peripheriques.append(sortie)
        return sortie

    try:
        moteur_gauche = Moteur(
            creer_sortie(PWMOutputDevice, MOTEUR_GAUCHE_PWM),
            creer_sortie(DigitalOutputDevice, MOTEUR_GAUCHE_IN1),
            creer_sortie(DigitalOutputDevice, MOTEUR_GAUCHE_IN2),
        )
        moteur_droit = Moteur(
            creer_sortie(PWMOutputDevice, MOTEUR_DROIT_PWM),
            creer_sortie(DigitalOutputDevice, MOTEUR_DROIT_IN1),
            creer_sortie(DigitalOutputDevice, MOTEUR_DROIT_IN2),
        )
        robot = Robot(moteur_gauche, moteur_droit)
        bus = SMBus(1)
        peripheriques.append(bus)
        imu = IMU(ICM20948(i2c_bus=bus))
        controleur = CtrlRobot(port_no, robot, imu, journal=journal)
        controleur.bus_i2c = bus
        return controleur
    except BaseException:
        for peripherique in reversed(peripheriques):
            peripherique.close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", help="Enregistrer les mesures brutes et corrigees dans ce CSV")
    options = parser.parse_args()
    try:
        with ExitStack() as fichiers:
            journal = (fichiers.enter_context(open(options.journal, "w", newline="", encoding="utf-8"))
                       if options.journal else None)
            controleur = creer_controleur(journal=journal)
            print(f"Controleur du robot en ecoute sur le port {APP_CTRL_ROBOT}.")
            controleur.run()
    except (OSError, RuntimeError, ValueError) as erreur:
        raise SystemExit(str(erreur)) from erreur


if __name__ == "__main__":
    main()
