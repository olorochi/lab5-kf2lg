#!/bin/python3
"""Aller-retour continu : avancer de 1 m, arreter, pivoter de 180 degres."""

import math
import time
from collections import deque

from ev_app import EvApp
from ev_app_client_api import gen_ev_externe
from param import (
    ANGLE_APPROCHE_DEMI_TOUR, ANGLE_DEMI_TOUR, APP_CTRL_ROBOT, APP_PATROUILLEUR,
    DELAI_ATTENTE_IMU, DELAI_AVANCE_MAX, DELAI_COMMUNICATION,
    DELAI_ROTATION_MAX, DISTANCE_PATROUILLE, DUREE_ARRET,
    FENETRE_BIAIS, GAIN_CORRECTION_CAP,
    IMPULSION_CORRECTION_MAX, IMPULSION_CORRECTION_MIN,
    INTERVALLE_AFFICHAGE_IMU, INTERVALLE_COMMANDE, INTERVALLE_CONTROLE, IP_CTRL_ROBOT,
    MSG_ARRETER, MSG_AVANCER, MSG_IMU_INFO, MSG_PIVOTER_D, MSG_PIVOTER_G,
    NB_CORRECTIONS_CAP_MAX, TEMPS_STABILISATION, TOLERANCE_CAP,
    TOLERANCE_REMISE_A_ZERO_ANGLE, TOLERANCE_REMISE_A_ZERO_DISTANCE,
    VERIFIER_CAP_DEMI_TOUR, VITESSE_ANGULAIRE_INITIALE,
)


class Patrouilleur(EvApp):
    def __init__(self, port_no=APP_PATROUILLEUR, horloge=time.monotonic,
                 envoyer=gen_ev_externe, verifier_cap=VERIFIER_CAP_DEMI_TOUR, **app_options):
        app_options.setdefault("tmo", INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.horloge = horloge
        self.envoyer = envoyer
        self.verifier_cap = verifier_cap
        self.etat = "ATTENTE_IMU"
        self.commande = MSG_ARRETER
        self._debut_etat = horloge()
        self._derniere_info = None
        self._prochaine_commande = horloge()
        self._prochain_affichage = horloge()
        self._arret_confirme = None
        self._apres_arret = MSG_AVANCER
        self.nombre_trajets = 0
        self._caps_repos = deque(maxlen=FENETRE_BIAIS)
        self._cap_cible = None
        self._nombre_corrections = 0
        self._vitesse_angulaire = VITESSE_ANGULAIRE_INITIALE
        self._angle_precedent = 0.0
        self._temps_angle = horloge()
        self._fin_impulsion = None
        print("En attente de la fin de calibration du controleur...")
        if not self.verifier_cap:
            print("Demi-tour valide par le gyroscope a 180 degres; cap magnetique indicatif.")

    def _envoyer_commande(self):
        self.envoyer(IP_CTRL_ROBOT, APP_CTRL_ROBOT, self.commande)
        self._prochaine_commande = self.horloge() + INTERVALLE_COMMANDE

    def _demander_arret(self, prochaine_commande):
        self.commande = MSG_ARRETER
        self.etat = "ARRET"
        self._debut_etat = self.horloge()
        self._arret_confirme = None
        self._apres_arret = prochaine_commande
        self._caps_repos.clear()
        self._fin_impulsion = None
        self.tmo = INTERVALLE_CONTROLE
        self._envoyer_commande()

    @staticmethod
    def erreur_cap(cible, mesure):
        return (cible - mesure + 180) % 360 - 180

    def _cap_moyen_repos(self):
        sinus = sum(math.sin(math.radians(c)) for c in self._caps_repos)
        cosinus = sum(math.cos(math.radians(c)) for c in self._caps_repos)
        return math.degrees(math.atan2(sinus, cosinus)) % 360

    def _lancer_mouvement(self, commande, cap, maintenant):
        self.commande = commande
        self.etat = "AVANCE" if commande == MSG_AVANCER else "PIVOT"
        self._debut_etat = maintenant
        self._prochain_affichage = maintenant
        if self.etat == "PIVOT":
            self._cap_cible = (cap + ANGLE_DEMI_TOUR) % 360
            self._nombre_corrections = 0
            self._vitesse_angulaire = VITESSE_ANGULAIRE_INITIALE
            self._angle_precedent = 0.0
            self._temps_angle = maintenant
        self._envoyer_commande()
        print(f"{self.etat} : cap de depart {cap:.1f} degres."
              + (f" Cible magnetique {self._cap_cible:.1f}."
                 if self.etat == "PIVOT" and self.verifier_cap else ""))

    def _verifier_demi_tour(self, cap, maintenant):
        erreur = self.erreur_cap(self._cap_cible, cap)
        print(f"Verification moteurs arretes : cap={cap:.1f}, "
              f"cible={self._cap_cible:.1f}, ecart={erreur:+.1f} degres.")
        if abs(erreur) <= TOLERANCE_CAP:
            self._cap_cible = None
            self._lancer_mouvement(MSG_AVANCER, cap, maintenant)
        elif self._nombre_corrections >= NB_CORRECTIONS_CAP_MAX:
            self._abandonner("Cap cible non atteint : verifier le compas et le montage.")
        else:
            duree = GAIN_CORRECTION_CAP * abs(erreur) / self._vitesse_angulaire
            duree = max(IMPULSION_CORRECTION_MIN, min(IMPULSION_CORRECTION_MAX, duree))
            self.commande = MSG_PIVOTER_D if erreur > 0 else MSG_PIVOTER_G
            self.etat = "CORRECTION"
            self._debut_etat = maintenant
            self._fin_impulsion = maintenant + duree
            self._nombre_corrections += 1
            self.tmo = min(INTERVALLE_CONTROLE, duree)
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
            if (abs(angle_rot) > TOLERANCE_REMISE_A_ZERO_ANGLE
                    or abs(dist_y) > TOLERANCE_REMISE_A_ZERO_DISTANCE):
                self._arret_confirme = None
                self._caps_repos.clear()
            elif self._arret_confirme is None:
                self._arret_confirme = maintenant
            else:
                if maintenant - self._arret_confirme >= TEMPS_STABILISATION:
                    self._caps_repos.append(cap_mag)
                if maintenant - self._arret_confirme >= DUREE_ARRET:
                    cap_repos = self._cap_moyen_repos()
                    if self._apres_arret is None:
                        self._verifier_demi_tour(cap_repos, maintenant)
                    else:
                        self._lancer_mouvement(self._apres_arret, cap_repos, maintenant)
        elif self.etat == "AVANCE" and distance >= DISTANCE_PATROUILLE:
            self.nombre_trajets += 1
            print(f"Trajet {self.nombre_trajets} : {distance:.3f} m "
                  f"(dist_y={dist_y:+.3f}). Arret avant demi-tour.")
            self._demander_arret(MSG_PIVOTER_G)
        elif self.etat == "PIVOT":
            dt = maintenant - self._temps_angle
            if dt > 0 and angle_rot != self._angle_precedent:
                self._vitesse_angulaire = max(10.0, abs(angle_rot - self._angle_precedent) / dt)
            self._angle_precedent = angle_rot
            self._temps_angle = maintenant
            seuil = ANGLE_APPROCHE_DEMI_TOUR if self.verifier_cap else ANGLE_DEMI_TOUR
            if abs(angle_rot) >= seuil:
                if self.verifier_cap:
                    print(f"Approche du demi-tour : {angle_rot:.1f} degres. "
                          "Arret pour verifier le cap et corriger par impulsions.")
                    self._demander_arret(None)
                else:
                    print(f"Demi-tour valide par le gyroscope : {angle_rot:.1f} degres. "
                          "Arret avant la reprise de l'avance.")
                    self._cap_cible = None
                    self._demander_arret(MSG_AVANCER)

    def traiter_cycle(self):
        maintenant = self.horloge()
        if self.etat == "TERMINE":
            return
        if self.etat == "CORRECTION":
            if maintenant >= self._fin_impulsion:
                self._demander_arret(None)
            else:
                self.tmo = min(INTERVALLE_CONTROLE, self._fin_impulsion - maintenant)
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
