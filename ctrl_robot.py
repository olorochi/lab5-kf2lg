#!/bin/python3

"""Arbitre la telecommande et le suiveur, puis commande les moteurs."""

import math
import time

from ev_app import EvApp
from mode_robot import FiltreMode
from moteur import Moteur
from param import (
    APP_CTRL_ROBOT,
    DELAI_SECURITE_COMMANDE,
    INTERVALLE_CONTROLE,
    MOTEUR_DROIT_IN1,
    MOTEUR_DROIT_IN2,
    MOTEUR_DROIT_PWM,
    MOTEUR_GAUCHE_IN1,
    MOTEUR_GAUCHE_IN2,
    MOTEUR_GAUCHE_PWM,
    MSG_ARRETER,
    MSG_AVANCER,
    MSG_COMMANDE_SUIVEUR,
    MSG_MODE_SUIVEUR,
    MSG_MODE_TELECOMMANDE,
    MSG_PIVOTER_D,
    MSG_PIVOTER_G,
    MSG_RECULER,
    MSG_VITESSE,
    VITESSE_MAX,
    VITESSE_MIN,
)
from robot import Robot


class CtrlRobot(EvApp):
    ACTIONS_SUIVEUR = frozenset(
        (MSG_AVANCER, MSG_PIVOTER_G, MSG_PIVOTER_D, MSG_ARRETER)
    )

    def __init__(self, port_no, robot, horloge=time.monotonic, **app_options):
        app_options.setdefault("tmo", INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.robot = robot
        self._horloge = horloge
        self._filtre_mode = FiltreMode()
        self.mode = MSG_MODE_TELECOMMANDE
        self.action = MSG_ARRETER
        self._derniere_commande = None
        self._actions = {
            MSG_ARRETER: self.robot.arreter,
            MSG_AVANCER: self.robot.avancer,
            MSG_RECULER: self.robot.reculer,
            MSG_PIVOTER_G: self.robot.pivoter_gauche,
            MSG_PIVOTER_D: self.robot.pivoter_droite,
        }
        self.robot.arreter()

    @staticmethod
    def lire_vitesse(evenement):
        donnees = evenement.split()
        if len(donnees) != 1 or not donnees[0]:
            raise ValueError("Une seule valeur de vitesse est requise")
        vitesse = float(donnees[0])
        if not math.isfinite(vitesse):
            raise ValueError("La vitesse doit etre finie")
        return max(VITESSE_MIN, min(VITESSE_MAX, vitesse))

    @classmethod
    def lire_commande_suiveur(cls, evenement):
        donnees = evenement.split()
        if len(donnees) != 1 or not donnees[0]:
            raise ValueError("Une seule commande de suiveur est requise")
        commande = int(donnees[0])
        if commande not in cls.ACTIONS_SUIVEUR:
            raise ValueError("Commande de suiveur interdite")
        return commande

    def _arreter(self):
        self.robot.arreter()
        self.action = MSG_ARRETER
        self._derniere_commande = None

    def _appliquer_mouvement(self, commande):
        changement = commande != self.action
        self._actions[commande]()
        self.action = commande
        self._derniere_commande = self._horloge()
        if changement:
            print(f"Commande: {commande}, PWM: {self.robot.vitesse:.2f}.")

    def _traiter_commande(self, evenement):
        if evenement.type in (MSG_MODE_TELECOMMANDE, MSG_MODE_SUIVEUR):
            if not self._filtre_mode.accepter(evenement):
                return
            if evenement.type != self.mode:
                self._arreter()
                self.mode = evenement.type
                print(f"Mode: {self.mode}.")
            return

        if evenement.type == MSG_VITESSE:
            self.robot.vitesse = self.lire_vitesse(evenement)
            self._actions[self.action]()
            print(f"Vitesse PWM: {self.robot.vitesse:.2f}.")
            return

        if evenement.type == MSG_COMMANDE_SUIVEUR:
            if self.mode == MSG_MODE_SUIVEUR:
                self._appliquer_mouvement(self.lire_commande_suiveur(evenement))
            return

        if evenement.type == MSG_ARRETER:
            self._appliquer_mouvement(MSG_ARRETER)
            return

        if self.mode == MSG_MODE_TELECOMMANDE and evenement.type in self._actions:
            self._appliquer_mouvement(evenement.type)

    def _verifier_delai(self):
        if self.action == MSG_ARRETER or self._derniere_commande is None:
            return
        if self._horloge() - self._derniere_commande >= DELAI_SECURITE_COMMANDE:
            self._arreter()
            print("Commandes interrompues; robot arrete.")

    def dispatch_event(self, evenement):
        self._verifier_delai()
        if evenement is None:
            return
        try:
            self._traiter_commande(evenement)
        except (TypeError, ValueError) as erreur:
            self._arreter()
            print(f"Commande invalide, robot arrete: {erreur}")

    def quitter(self):
        self.action = MSG_ARRETER
        self._derniere_commande = None
        self.robot.fermer()
        print("Controleur arrete; moteurs desactives.")


def creer_controleur(port_no=APP_CTRL_ROBOT):
    try:
        from gpiozero import DigitalOutputDevice, PWMOutputDevice
    except ImportError as erreur:
        raise SystemExit("gpiozero est necessaire sur le Raspberry Pi") from erreur

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
        return CtrlRobot(port_no, robot)
    except BaseException:
        for peripherique in reversed(peripheriques):
            peripherique.close()
        raise


def main():
    controleur = creer_controleur()
    print(f"Controleur du robot en ecoute sur le port {APP_CTRL_ROBOT}.")
    controleur.run()


if __name__ == "__main__":
    main()
