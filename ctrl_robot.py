#!/bin/python3

"""Controleur du robot."""

import math
import time

from ev_app import EvApp
from ev_app_client_api import gen_ev_externe
from imu import IMU
from moteur import Moteur
from param import (
    APP_CTRL_ROBOT,
    APP_PATROUILLEUR,
    DELAI_CALIBRATION_MAX,
    DELAI_COMMUNICATION,
    DUREE_ARRET,
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
    TEMPS_STABILISATION,
    VITESSE_MAX,
    VITESSE_MIN,
)
from robot import Robot


class CtrlRobot(EvApp):
    def __init__(self, port_no, robot, imu, horloge=time.monotonic,
                 envoyer=gen_ev_externe, calibrer=True, **app_options):
        app_options.setdefault("tmo", INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.robot = robot
        self.imu = imu
        self.horloge = horloge
        self.envoyer = envoyer
        maintenant = horloge()
        self._dernier_echantillon = maintenant
        self._prochain_echantillon = maintenant + INTERVALLE_CONTROLE
        self._derniere_commande = maintenant
        self._biais_apres = maintenant
        self._phase = "BIAIS" if calibrer else "PRET"
        self._debut_phase = maintenant
        self._angle_calibration = 0.0
        self._gx_calibration_precedent = 0.0
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
            print("Calibration : laisser le robot immobile pour mesurer les biais.")

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

    def _calibrer(self, maintenant, dt):
        if self._phase == "BIAIS" and self.imu.biais_pret:
            self.imu.commencer_calibration_magnetometre()
            self._phase = "MAGNETOMETRE"
            self._debut_phase = maintenant
            self.robot.pivoter_gauche()
            print(f"Calibration magnetique : {NB_TOURS_CALIBRATION} tours sur place.")
        elif self._phase == "MAGNETOMETRE":
            if maintenant - self._debut_phase > DELAI_CALIBRATION_MAX:
                raise RuntimeError("Calibration trop longue : verifier le gyroscope et les moteurs")
            gx = self.imu.vitesse_angulaire
            self._angle_calibration += dt * (gx + self._gx_calibration_precedent) / 2
            self._gx_calibration_precedent = gx
            if abs(self._angle_calibration) >= NB_TOURS_CALIBRATION * 360:
                self._arreter()
                self.imu.terminer_calibration_magnetometre()
                self._phase = "REPOS"
                self._debut_phase = maintenant
        elif self._phase == "REPOS" and maintenant - self._debut_phase >= DUREE_ARRET:
            self._phase = "PRET"
            print(f"Calibration terminee : biais ay={self.imu.ay_biais:.6f} g, "
                  f"gx={self.imu.gx_biais:.4f} deg/s; "
                  f"corrections my={self.imu.corr_my:.3f}, mz={self.imu.corr_mz:.3f}.")

    def traiter_cycle(self):
        maintenant = self.horloge()
        if (self._phase == "PRET" and self.action != MSG_ARRETER
                and maintenant - self._derniere_commande > DELAI_COMMUNICATION):
            self._arreter()
            raise RuntimeError("Patrouilleur sans nouvelles : robot arrete")
        if maintenant >= self._prochain_echantillon:
            dt = maintenant - self._dernier_echantillon
            if dt > INTERVALLE_IMU_MAX:
                raise RuntimeError("Echantillonnage IMU interrompu : robot arrete")
            self._dernier_echantillon = maintenant
            self._prochain_echantillon = maintenant + INTERVALLE_CONTROLE
            actif = self._phase == "PRET" and self.action != MSG_ARRETER
            angle_rot, dist_y = self.imu.obtenir_deplacement(
                actif, dt, calibrer_biais=(self._phase != "MAGNETOMETRE"
                                         and maintenant >= self._biais_apres),
            )
            cap_mag = self.imu.obtenir_cap_magnetique()
            if self._phase != "PRET":
                self._calibrer(maintenant, dt)
            if self._phase == "PRET":
                self.envoyer(IP_PATROUILLEUR, APP_PATROUILLEUR, MSG_IMU_INFO,
                             angle_rot, dist_y, cap_mag)
        self.tmo = max(0.0, self._prochain_echantillon - self.horloge())

    def quitter(self):
        self.action = MSG_ARRETER
        try:
            self.robot.fermer()
        finally:
            if hasattr(self, "bus_i2c"):
                self.bus_i2c.close()
        print("Controleur arrete; moteurs desactives.")


def creer_controleur(port_no=APP_CTRL_ROBOT):
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
        controleur = CtrlRobot(port_no, robot, imu)
        controleur.bus_i2c = bus
        return controleur
    except BaseException:
        for peripherique in reversed(peripheriques):
            peripherique.close()
        raise


def main():
    try:
        controleur = creer_controleur()
        print(f"Controleur du robot en ecoute sur le port {APP_CTRL_ROBOT}.")
        controleur.run()
    except (OSError, RuntimeError, ValueError) as erreur:
        raise SystemExit(str(erreur)) from erreur


if __name__ == "__main__":
    main()
