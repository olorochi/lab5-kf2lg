#!/bin/python3
"""Aller-retour continu : avancer de 1 m, arreter, pivoter de 180 degres."""

import math
import time

from ev_app import EvApp
from ev_app_client_api import gen_ev_externe
from param import (
    ANGLE_DEMI_TOUR, APP_CTRL_ROBOT, APP_PATROUILLEUR,
    DELAI_ATTENTE_IMU, DELAI_AVANCE_MAX, DELAI_COMMUNICATION,
    DELAI_ROTATION_MAX, DISTANCE_PATROUILLE, DUREE_ARRET,
    INTERVALLE_AFFICHAGE_IMU, INTERVALLE_COMMANDE, INTERVALLE_CONTROLE, IP_CTRL_ROBOT,
    MSG_ARRETER, MSG_AVANCER, MSG_IMU_INFO, MSG_PIVOTER_G,
)


class Patrouilleur(EvApp):
    def __init__(self, port_no=APP_PATROUILLEUR, horloge=time.monotonic,
                 envoyer=gen_ev_externe, **app_options):
        app_options.setdefault("tmo", INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.horloge = horloge
        self.envoyer = envoyer
        self.etat = "ATTENTE_IMU"
        self.commande = MSG_ARRETER
        self._debut_etat = horloge()
        self._derniere_info = None
        self._prochaine_commande = horloge()
        self._prochain_affichage = horloge()
        self._arret_confirme = None
        self._apres_arret = MSG_AVANCER
        self.nombre_trajets = 0
        print("En attente de la fin de calibration du controleur...")

    def _envoyer_commande(self):
        self.envoyer(IP_CTRL_ROBOT, APP_CTRL_ROBOT, self.commande)
        self._prochaine_commande = self.horloge() + INTERVALLE_COMMANDE

    def _demander_arret(self, prochaine_commande):
        self.commande = MSG_ARRETER
        self.etat = "ARRET"
        self._debut_etat = self.horloge()
        self._arret_confirme = None
        self._apres_arret = prochaine_commande
        self._envoyer_commande()

    def _abandonner(self, raison):
        print(raison)
        self.commande = MSG_ARRETER
        self.etat = "TERMINE"
        self._envoyer_commande()
        self.quitter_app()

    def dispatch_event(self, evenement):
        if evenement is None or evenement.type != MSG_IMU_INFO or self._termine:
            return
        try:
            donnees = evenement.split()
            if len(donnees) != 3:
                raise ValueError("MSG_IMU_INFO attend angle_rot;dist_y;cap_mag")
            angle_rot, dist_y, cap_mag = map(float, donnees)
            if not all(map(math.isfinite, (angle_rot, dist_y, cap_mag))):
                raise ValueError("Les mesures IMU doivent etre finies")
            if not 0 <= cap_mag < 360:
                raise ValueError("Le cap doit etre compris entre 0 et 359 degres")
        except (TypeError, ValueError) as erreur:
            self._abandonner(f"Mesures invalides, arret : {erreur}")
            return

        maintenant = self.horloge()
        self._derniere_info = maintenant
        distance = abs(dist_y)
        if self.etat in ("AVANCE", "PIVOT") and maintenant >= self._prochain_affichage:
            print(f"{self.etat} : distance={distance:.3f}/{DISTANCE_PATROUILLE:.3f} m, "
                  f"dist_y={dist_y:+.3f} m, angle={angle_rot:+.1f} degres, "
                  f"cap={cap_mag:.0f}.")
            self._prochain_affichage = maintenant + INTERVALLE_AFFICHAGE_IMU
        if self.etat == "ATTENTE_IMU":
            self._demander_arret(MSG_AVANCER)
        elif self.etat == "ARRET":
            if abs(angle_rot) > 1e-9 or abs(dist_y) > 1e-9:
                self._arret_confirme = None
            elif self._arret_confirme is None:
                self._arret_confirme = maintenant
            elif maintenant - self._arret_confirme >= DUREE_ARRET:
                self.commande = self._apres_arret
                self.etat = "AVANCE" if self.commande == MSG_AVANCER else "PIVOT"
                self._debut_etat = maintenant
                self._prochain_affichage = maintenant
                self._envoyer_commande()
                print(f"{self.etat} : cap magnetique {cap_mag:.0f} degres.")
        elif self.etat == "AVANCE" and distance >= DISTANCE_PATROUILLE:
            self.nombre_trajets += 1
            print(f"Trajet {self.nombre_trajets} : {distance:.3f} m "
                  f"(dist_y={dist_y:+.3f}). Arret avant demi-tour.")
            self._demander_arret(MSG_PIVOTER_G)
        elif self.etat == "PIVOT" and abs(angle_rot) >= ANGLE_DEMI_TOUR:
            print(f"Demi-tour : {angle_rot:.1f} degres, cap {cap_mag:.0f}. Arret avant avance.")
            self._demander_arret(MSG_AVANCER)

    def traiter_cycle(self):
        maintenant = self.horloge()
        if self.etat == "TERMINE":
            return
        if self.etat == "ATTENTE_IMU":
            if maintenant - self._debut_etat > DELAI_ATTENTE_IMU:
                self._abandonner("Aucune mesure IMU : verifier que ctrl_robot.py est lance.")
            return
        if maintenant - self._derniere_info > DELAI_COMMUNICATION:
            self._abandonner("Mesures IMU interrompues : robot arrete.")
            return
        limite = {"AVANCE": DELAI_AVANCE_MAX, "PIVOT": DELAI_ROTATION_MAX,
                  "ARRET": DUREE_ARRET + DELAI_COMMUNICATION}
        if maintenant - self._debut_etat > limite[self.etat]:
            self._abandonner(f"Delai depasse dans l'etat {self.etat} : verifier les mesures.")
            return
        if maintenant >= self._prochaine_commande:
            self._envoyer_commande()

    def quitter(self):
        self.commande = MSG_ARRETER
        self._envoyer_commande()
        print("Patrouilleur arrete; commande d'arret envoyee au robot.")


def main():
    try:
        Patrouilleur().run()
    except OSError as erreur:
        raise SystemExit(f"Communication du patrouilleur : {erreur}") from erreur


if __name__ == "__main__":
    main()
