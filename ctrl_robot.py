#!/bin/python3
"""Controle moteur, integration IMU rapide et rotations."""

import argparse
import csv
import math
import time
from contextlib import ExitStack
from ev_app import EvApp
from ev_app_client_api import gen_ev_externe
from imu import IMU
from moteur import Moteur
from rotation import RotationControlee
from robot import Robot
import param as p


class CtrlRobot(EvApp):
    def __init__(self, port_no, robot, imu, horloge=time.monotonic,
                 envoyer=gen_ev_externe, calibrer=True, journal=None, **app_options):
        app_options.setdefault("tmo", p.INTERVALLE_INTEGRATION)
        super().__init__(port_no, **app_options)
        self.robot, self.imu, self.horloge, self.envoyer = robot, imu, horloge, envoyer
        maintenant = horloge()
        self._dernier_echantillon = maintenant
        self._prochain_echantillon = maintenant+p.INTERVALLE_INTEGRATION
        self._prochaine_info = maintenant+p.ICM20948_DT
        self._derniere_commande = maintenant
        self._biais_apres = maintenant
        self._debut_phase = maintenant
        self._phase = "BIAIS" if calibrer else "PRET"
        self.action = p.MSG_ARRETER
        self.rotation = None
        self.rotation_id = None
        self.cap_initial = None
        self.sens_cap_gauche, self.sens_gyro_gauche = -1, 1
        self.vitesse_cible = p.VITESSE_INITIALE
        self._pwm_avance = 0.0
        self._gyro_avance = 0.0
        self._derniere_distance = 0.0
        self._dernier_dt = 0.0
        self._journal = journal
        self._csv = csv.writer(journal) if journal is not None else None
        if self._csv:
            self._csv.writerow(("temps", "phase", "action", "dt", "ay_g", "biais_ay_g",
                                "acceleration_m_s2", "vy_m_s", "distance_m", "gyro_total_deg",
                                "cap_deg", "rotation_progression", "rotation_etat"))
        self.robot.arreter()
        print("Laisser le robot immobile pendant la mesure du biais.")

    @staticmethod
    def lire_vitesse(evenement):
        donnees = evenement.split()
        if len(donnees) != 1:
            raise ValueError("Une seule valeur de vitesse est requise")
        vitesse = float(donnees[0])
        if not math.isfinite(vitesse):
            raise ValueError("La vitesse doit etre finie")
        return max(p.VITESSE_MIN, min(p.VITESSE_MAX, vitesse))

    def _arreter(self):
        if self.rotation and not self.rotation.termine and self.rotation.etat != "ANNULEE":
            self.rotation.annuler(self.horloge())
        if self.action != p.MSG_ARRETER:
            self._biais_apres = self.horloge()+p.TEMPS_STABILISATION
            self.imu.vider_fenetre_biais()
        self.robot.arreter()
        self.action = p.MSG_ARRETER
        self._pwm_avance = 0.0
        self.robot.vitesse = self.vitesse_cible

    def dispatch_event(self, evenement):
        if evenement is None:
            return
        try:
            if self._phase != "PRET":
                if evenement.type == p.MSG_ARRETER:
                    self._arreter()
                    self.quitter_app()
                return
            if evenement.type == p.MSG_VITESSE:
                self.vitesse_cible = self.lire_vitesse(evenement)
                return
            if evenement.type not in (p.MSG_ARRETER, p.MSG_AVANCER, p.MSG_RECULER,
                                      p.MSG_PIVOTER_G, p.MSG_PIVOTER_D):
                return
            self._derniere_commande = self.horloge()
            if evenement.type == p.MSG_ARRETER:
                self._arreter()
                return
            if evenement.type == p.MSG_PIVOTER_G and evenement.donnee:
                donnees = evenement.split()
                if len(donnees) != 2:
                    raise ValueError("Rotation cible : angle;identifiant")
                angle, identifiant = float(donnees[0]), int(donnees[1])
                if not math.isfinite(angle) or not 0 < angle <= 1080 or identifiant < 1:
                    raise ValueError("Cible de rotation invalide")
                if self.rotation_id == identifiant:
                    return  # Les repetitions UDP ne redemarrent jamais la rotation.
                if self.rotation and self.rotation.en_mouvement:
                    raise ValueError("Une rotation est deja en cours")
                self.robot.vitesse = self.vitesse_cible
                self.rotation = RotationControlee(
                    self.robot, self.imu, angle, self.horloge(),
                    sens_cap_gauche=self.sens_cap_gauche, sens_gyro_gauche=self.sens_gyro_gauche)
                self.rotation_id = identifiant
                return
            if self.rotation and not self.rotation.termine and self.rotation.etat != "ANNULEE":
                self._arreter()
            self.rotation, self.rotation_id = None, None
            if evenement.type != self.action:
                self.imu.reinitialiser_deplacement()
                self._gyro_avance = self.imu.angle_total
                self._pwm_avance = 0.0
            self.action = evenement.type
            if self.action == p.MSG_RECULER:
                self.robot.vitesse = self.vitesse_cible
                self.robot.reculer()
            elif self.action == p.MSG_PIVOTER_G:
                self.robot.vitesse = self.vitesse_cible
                self.robot.pivoter_gauche()
            elif self.action == p.MSG_PIVOTER_D:
                self.robot.vitesse = self.vitesse_cible
                self.robot.pivoter_droite()
        except (ValueError, TypeError) as erreur:
            self._arreter()
            print(f"Commande invalide : {erreur}")

    def _avance_progressive(self, dt):
        cible = self.vitesse_cible
        if abs(self._derniere_distance) >= p.DISTANCE_PATROUILLE-p.DISTANCE_RALENTISSEMENT:
            cible = min(cible, p.PWM_AVANCE_LENTE)
        pas = p.RAMPE_PWM_PAR_SECONDE*dt
        self._pwm_avance += max(-pas, min(pas, cible-self._pwm_avance))
        self.robot.vitesse = self._pwm_avance
        signe = -self.sens_gyro_gauche
        ecart = signe*(self.imu.angle_total-self._gyro_avance)
        self.robot.avancer_corrige(0.008*ecart)

    def traiter_cycle(self):
        maintenant = self.horloge()
        objectif_actif = self.rotation and self.rotation.etat not in ("TERMINE", "ANNULEE")
        if (self._phase == "PRET" and (self.action != p.MSG_ARRETER or objectif_actif)
                and maintenant-self._derniere_commande > p.DELAI_COMMUNICATION):
            self._arreter()
            raise RuntimeError("Patrouilleur sans nouvelles : robot arrete")
        nouveau_cap = False
        dt = maintenant-self._dernier_echantillon
        if maintenant+1e-9 >= self._prochain_echantillon:
            if dt > p.INTERVALLE_IMU_MAX and (self.action != p.MSG_ARRETER or
                                            (self.rotation and self.rotation.en_mouvement)):
                self._arreter()
                raise RuntimeError(f"Echantillonnage interrompu en mouvement : dt={dt:.3f} s")
            self._dernier_echantillon = maintenant
            self._dernier_dt = dt
            self._prochain_echantillon = maintenant+p.INTERVALLE_INTEGRATION
            actif = (self._phase == "PRET" and
                     (self.rotation.en_mouvement if objectif_actif else self.action != p.MSG_ARRETER))
            mesurer_biais = (not objectif_actif and self._phase != "CALIBRATION"
                             and maintenant >= self._biais_apres)
            _, self._derniere_distance = self.imu.obtenir_deplacement(
                actif, dt, calibrer_biais=mesurer_biais)
            if self._phase == "PRET" and self.action == p.MSG_AVANCER and not objectif_actif:
                self._avance_progressive(dt)
        if maintenant+1e-9 >= self._prochaine_info:
            self._prochaine_info = maintenant+p.ICM20948_DT
            if self.rotation and self.rotation.calibration:
                self.imu.calibration_magnetometre = (
                    not self.rotation._calibration_finie and not self.rotation.en_mouvement
                    and maintenant-self.rotation.debut_repos >= p.TEMPS_STABILISATION)
            self.imu.obtenir_cap_magnetique()
            nouveau_cap = True
        if self._phase == "BIAIS" and self.imu.biais_pret and self.imu.magne_data is not None:
            self.robot.vitesse = self.vitesse_cible
            self.rotation = RotationControlee(self.robot, self.imu, p.NB_TOURS_CALIBRATION*360,
                                               maintenant, calibration=True)
            self._phase, self._debut_phase = "CALIBRATION", maintenant
            print(f"Calibration : {p.NB_TOURS_CALIBRATION} tours nets, cap de depart memorise.")
        if self.rotation and self.rotation.etat not in ("TERMINE", "ANNULEE"):
            limite = p.DELAI_CALIBRATION_MAX if self._phase == "CALIBRATION" else p.DELAI_ROTATION_MAX
            if maintenant-self.rotation.debut > limite:
                self._arreter()
                raise RuntimeError("Rotation non convergente : consulter cap, progression et montage")
            self.rotation.actualiser(maintenant, nouveau_cap)
            if self._phase == "PRET":
                self.action = self.rotation.commande
            if self.rotation.termine:
                self._biais_apres = maintenant+p.TEMPS_STABILISATION
                self.imu.vider_fenetre_biais()
                self.action = p.MSG_ARRETER
                if self._phase == "CALIBRATION":
                    self.cap_initial = self.rotation.cap_depart
                    self.sens_gyro_gauche = self.rotation.sens_gyro_gauche
                    self.sens_cap_gauche = self.rotation.sens_cap_gauche
                    self._phase, self._debut_phase = "REPOS", maintenant
                    print(f"Trois tours verifies : {self.rotation.progression:.1f} degres; "
                          f"cap initial {self.cap_initial:.1f}, cap final {self.imu.cap_precis:.1f}.")
                    self.rotation = None
        if self._phase == "REPOS" and self.imu.biais_pret and maintenant-self._debut_phase >= p.DUREE_ARRET:
            self._phase = "PRET"
            print(f"Controleur PRET. Depart {self.cap_initial:.1f} degres; "
                  f"biais ay={self.imu.ay_biais:.6f} g, gx={self.imu.gx_biais:.4f} deg/s.")
        if nouveau_cap and self._phase == "PRET":
            self.envoyer(p.IP_PATROUILLEUR, p.APP_PATROUILLEUR, p.MSG_IMU_INFO,
                         self.imu.angle_rot, self._derniere_distance, int(self.imu.cap_precis))
            if self.cap_initial is not None:
                self.envoyer(p.IP_PATROUILLEUR, p.APP_PATROUILLEUR, p.MSG_REFERENCE_CAP,
                             self.cap_initial, self.sens_cap_gauche)
            if self.rotation is not None and self.rotation_id is not None:
                self.envoyer(p.IP_PATROUILLEUR, p.APP_PATROUILLEUR, p.MSG_ROTATION_INFO,
                             self.rotation_id, self.rotation.etat, self.imu.cap_precis,
                             self.rotation.progression)
        if nouveau_cap and self._csv is not None:
            self._csv.writerow((maintenant, self._phase, self.action, self._dernier_dt, self.imu.accel_data[1],
                                self.imu.ay_biais, self.imu.acceleration_corrigee, self.imu.vy,
                                self._derniere_distance, self.imu.angle_total, self.imu.cap_precis,
                                self.rotation.progression if self.rotation else 0,
                                self.rotation.etat if self.rotation else ""))
            self._journal.flush()
        echeances = [self._prochain_echantillon, self._prochaine_info]
        if self.rotation and self.rotation.fin_impulsion is not None:
            echeances.append(self.rotation.fin_impulsion)
        self.tmo = max(0.0, min(echeances)-self.horloge())

    def quitter(self):
        self._arreter()
        try:
            self.robot.fermer()
        finally:
            if hasattr(self, "bus_i2c"):
                self.bus_i2c.close()
        print("Controleur arrete; moteurs desactives.")


def creer_controleur(port_no=p.APP_CTRL_ROBOT, journal=None):
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
    from icm20948 import ICM20948
    from smbus2 import SMBus
    from capteur_icm import CapteurICM
    peripheriques = []
    def sortie(classe, broche):
        appareil = classe(broche)
        peripheriques.append(appareil)
        return appareil
    try:
        gauche = Moteur(sortie(PWMOutputDevice, p.MOTEUR_GAUCHE_PWM),
                        sortie(DigitalOutputDevice, p.MOTEUR_GAUCHE_IN1),
                        sortie(DigitalOutputDevice, p.MOTEUR_GAUCHE_IN2))
        droit = Moteur(sortie(PWMOutputDevice, p.MOTEUR_DROIT_PWM),
                       sortie(DigitalOutputDevice, p.MOTEUR_DROIT_IN1),
                       sortie(DigitalOutputDevice, p.MOTEUR_DROIT_IN2))
        bus = SMBus(1)
        peripheriques.append(bus)
        capteur = CapteurICM(ICM20948(i2c_bus=bus))
        controleur = CtrlRobot(port_no, Robot(gauche, droit), IMU(capteur), journal=journal)
        controleur.bus_i2c = bus
        return controleur
    except BaseException:
        for appareil in reversed(peripheriques):
            appareil.close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", help="CSV des mesures pour verifier l'essai reel")
    options = parser.parse_args()
    try:
        with ExitStack() as fichiers:
            journal = (fichiers.enter_context(open(options.journal, "w", newline="", encoding="utf-8"))
                       if options.journal else None)
            controleur = creer_controleur(journal=journal)
            print(f"Controleur en ecoute sur le port {p.APP_CTRL_ROBOT}.")
            controleur.run()
    except (ImportError, RuntimeError, OSError, ValueError) as erreur:
        raise SystemExit(str(erreur)) from erreur


if __name__ == "__main__":
    main()
