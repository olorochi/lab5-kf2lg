#!/bin/python3
"""Patrouille continue : un metre puis un demi-tour."""

import math
import time
from ev_app import EvApp
from ev_app_client_api import gen_ev_externe
import param as p


class Patrouilleur(EvApp):
    def __init__(self, port_no=p.APP_PATROUILLEUR, horloge=time.monotonic,
                 envoyer=gen_ev_externe, **app_options):
        app_options.setdefault("tmo", p.INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.horloge, self.envoyer = horloge, envoyer
        self.etat, self.commande = "ATTENTE_IMU", p.MSG_ARRETER
        self._debut_etat = horloge()
        self._derniere_info = None
        self._prochaine_commande = horloge()
        self._arret_confirme = None
        self._apres_arret = p.MSG_AVANCER
        self._rotation_id = 0
        self._dernier_cap = None
        self.cap_initial = None
        self.cap_depart = None
        self._distance_precedente = 0.0
        self._temps_distance = None
        self._vitesse = 0.0
        self.nombre_trajets = 0
        self._prochain_affichage = horloge()
        print("Attente de la calibration des trois tours...")

    def _envoyer_commande(self):
        if self.etat == "PIVOT":
            self.envoyer(p.IP_CTRL_ROBOT, p.APP_CTRL_ROBOT, p.MSG_PIVOTER_G,
                         p.ANGLE_DEMI_TOUR, self._rotation_id)
        else:
            self.envoyer(p.IP_CTRL_ROBOT, p.APP_CTRL_ROBOT, self.commande)
        self._prochaine_commande = self.horloge()+p.INTERVALLE_COMMANDE

    def _demander_arret(self, prochaine_commande):
        self.commande, self.etat = p.MSG_ARRETER, "ARRET"
        self._debut_etat = self.horloge()
        self._arret_confirme = None
        self._apres_arret = prochaine_commande
        self._envoyer_commande()

    def _abandonner(self, raison):
        print(raison)
        self.commande, self.etat = p.MSG_ARRETER, "TERMINE"
        self._envoyer_commande()
        self.quitter_app()

    def _rotation_info(self, evenement):
        valeurs = evenement.split()
        if len(valeurs) != 4:
            raise ValueError("Etat rotation : identifiant;etat;cap;progression")
        identifiant, etat = int(valeurs[0]), valeurs[1]
        cap, progression = map(float, valeurs[2:])
        if not all(map(math.isfinite, (cap, progression))):
            raise ValueError("Etat de rotation non fini")
        if self.etat != "PIVOT" or identifiant != self._rotation_id:
            return
        if etat == "TERMINE":
            print(f"Demi-tour mesure {progression:.1f} degres; cap final {cap:.1f}.")
            self._demander_arret(p.MSG_AVANCER)
        elif etat == "ANNULEE":
            self._abandonner("Rotation annulee par le controleur.")

    def dispatch_event(self, evenement):
        if evenement is None or self._termine:
            return
        try:
            if evenement.type == p.MSG_REFERENCE_CAP:
                valeurs = evenement.split()
                if len(valeurs) != 2:
                    raise ValueError("Reference de cap invalide")
                cap, sens = map(float, valeurs)
                if not math.isfinite(cap) or not 0 <= cap < 360 or sens not in (-1,1):
                    raise ValueError("Reference de cap invalide")
                if self.cap_initial is None:
                    self.cap_initial = cap
                    print(f"Cap initial avant les trois tours : {cap:.1f} degres.")
                return
            if evenement.type == p.MSG_ROTATION_INFO:
                self._rotation_info(evenement)
                return
            if evenement.type != p.MSG_IMU_INFO:
                return
            valeurs = evenement.split()
            if len(valeurs) != 3:
                raise ValueError("MSG_IMU_INFO attend angle_rot;dist_y;cap_mag")
            angle, distance_signee, cap = map(float, valeurs)
            if not all(map(math.isfinite, (angle,distance_signee,cap))) or not 0 <= cap < 360:
                raise ValueError("Mesures IMU invalides")
        except (TypeError, ValueError) as erreur:
            self._abandonner(f"Donnees invalides : {erreur}")
            return
        maintenant = self.horloge()
        self._derniere_info, self._dernier_cap = maintenant, cap
        distance = abs(distance_signee)
        if self.etat == "ATTENTE_IMU":
            self._demander_arret(p.MSG_AVANCER)
        elif self.etat == "ARRET":
            if abs(angle) > 1.0 or distance > 0.01:
                self._arret_confirme = None
            elif self._arret_confirme is None:
                self._arret_confirme = maintenant
            elif maintenant-self._arret_confirme >= p.DUREE_ARRET:
                self.commande = self._apres_arret
                self._debut_etat = maintenant
                if self.commande == p.MSG_AVANCER:
                    self.etat = "AVANCE"
                    if self.cap_depart is None:
                        self.cap_depart = cap
                        print(f"Direction au depart de la patrouille : {cap:.1f} degres.")
                    self._distance_precedente = 0.0
                    self._temps_distance = maintenant
                    self._vitesse = 0.0
                else:
                    self.etat = "PIVOT"
                    self._rotation_id += 1
                    print(f"Demi-tour #{self._rotation_id} depuis le cap {cap:.1f} degres.")
                self._envoyer_commande()
        elif self.etat == "AVANCE":
            dt = maintenant-self._temps_distance
            if dt > 0:
                mesure = max(0.0, min(2.0, (distance-self._distance_precedente)/dt))
                alpha = dt/(0.15+dt)
                self._vitesse += alpha*(mesure-self._vitesse)
            self._temps_distance, self._distance_precedente = maintenant, distance
            anticipation = self._vitesse*(p.TEMPS_ARRET_ESTIME+p.ICM20948_DT+p.FILTRE_ACCEL_TAU)
            if maintenant >= self._prochain_affichage:
                print(f"AVANCE : distance estimee {distance:.3f} m; "
                      f"vitesse {self._vitesse:.3f} m/s; cap {cap:.1f}.")
                self._prochain_affichage = maintenant+p.INTERVALLE_AFFICHAGE_IMU
            proche = distance >= p.DISTANCE_PATROUILLE-p.DISTANCE_RALENTISSEMENT
            if distance >= p.DISTANCE_PATROUILLE or (proche and
                    distance+anticipation >= p.DISTANCE_PATROUILLE-p.TOLERANCE_DISTANCE):
                self.nombre_trajets += 1
                print(f"Trajet {self.nombre_trajets} : arret a {distance:.3f} m estimes "
                      f"(+{anticipation:.3f} m de reponse anticipee).")
                self._demander_arret(p.MSG_PIVOTER_G)

    def traiter_cycle(self):
        if self.etat == "TERMINE":
            return
        maintenant = self.horloge()
        if self.etat == "ATTENTE_IMU":
            if maintenant-self._debut_etat > p.DELAI_ATTENTE_IMU:
                self._abandonner("Controleur sans mesures apres le delai de calibration.")
            return
        if maintenant-self._derniere_info > p.DELAI_COMMUNICATION:
            self._abandonner("Mesures IMU interrompues : arret.")
            return
        limite = {"AVANCE":p.DELAI_AVANCE_MAX, "PIVOT":p.DELAI_ROTATION_MAX,
                  "ARRET":p.DUREE_ARRET+p.DELAI_COMMUNICATION}
        if maintenant-self._debut_etat > limite[self.etat]:
            self._abandonner(f"Delai depasse dans {self.etat} : verifier le journal IMU.")
            return
        if maintenant >= self._prochaine_commande:
            self._envoyer_commande()

    def quitter(self):
        self.etat, self.commande = "TERMINE", p.MSG_ARRETER
        self._envoyer_commande()
        print("Patrouilleur arrete.")


def main():
    try:
        Patrouilleur().run()
    except OSError as erreur:
        raise SystemExit(str(erreur)) from erreur


if __name__ == "__main__":
    main()
