#!/usr/bin/env python3

"""Service de poursuite : la camera du Pi commande le robot par UDP."""

import argparse
import time

from ev_app import EvApp
from ev_app_client_api import TAILLE_MSG_TOT, fermer_client, gen_ev_externe
from mode_robot import FiltreMode
from param import (
    AIRE_BALLE_MAX, AIRE_BALLE_MIN, APP_CTRL_ROBOT, APP_SUIVEUR,
    DELAI_SECURITE_TELECOMMANDE, INTERVALLE_CAMERA_MS, MARGE_CENTRE,
    MSG_ARRETER, MSG_AVANCER, MSG_COMMANDE_SUIVEUR, MSG_MODE_SUIVEUR,
    MSG_MODE_TELECOMMANDE, MSG_PIVOTER_D, MSG_PIVOTER_G, TEINTE_MAX, TEINTE_MIN,
)

FENETRE_IMAGE = "Suiveur - camera Pi"
FENETRE_MASQUE = "Suiveur - masque HSV"


def choisir_commande(aire, centre_x, largeur, aire_min=AIRE_BALLE_MIN, aire_max=AIRE_BALLE_MAX, marge=MARGE_CENTRE):
    """Priorite a l'arret hors distance, puis a l'alignement horizontal."""
    if aire <= 0 or aire < aire_min or aire > aire_max:
        return MSG_ARRETER
    if centre_x < largeur / 2 - marge:
        return MSG_PIVOTER_G
    if centre_x > largeur / 2 + marge:
        return MSG_PIVOTER_D
    return MSG_AVANCER


class Suiveur(EvApp):
    def __init__(self, port_no=APP_SUIVEUR, ip_robot="127.0.0.1", detecteur=None, envoyer=gen_ev_externe, horloge=time.monotonic, affichage=True, **app_options):
        if detecteur is None:
            from detecteur_hsv import Detecteur_hsv
            detecteur = Detecteur_hsv()
        super().__init__(port_no, tmo=INTERVALLE_CAMERA_MS / 1000, **app_options)
        self.detecteur = detecteur
        self.ip_robot = ip_robot
        self.affichage = affichage
        self.actif = False
        self._bloque = False
        self._dernier_contact = None
        self._attente_contact = False
        self._debut_attente = None
        self._envoyer = envoyer
        self._horloge = horloge
        self._filtre_mode = FiltreMode()

    def envoyer_commande(self, commande):
        self._envoyer(self.ip_robot, APP_CTRL_ROBOT, MSG_COMMANDE_SUIVEUR, commande)

    def desactiver(self):
        self.actif = False
        self._attente_contact = False
        try:
            self.envoyer_commande(MSG_ARRETER)
        finally:
            try:
                self.detecteur.arreter()
            finally:
                if self.affichage:
                    import cv2
                    cv2.destroyAllWindows()

    def _purger_modes_pendant_ouverture(self):
        while True:
            try:
                message, _ = self.udp_sock.recvfrom(TAILLE_MSG_TOT + 1)
            except BlockingIOError:
                return
            try:
                evenement = self.decoder_message(message)
                if evenement.type not in (MSG_MODE_TELECOMMANDE, MSG_MODE_SUIVEUR):
                    continue
                if not self._filtre_mode.accepter(evenement):
                    continue
            except (OSError, ValueError):
                continue
            if evenement.type == MSG_MODE_TELECOMMANDE:
                self.desactiver()
                self._bloque = False
                return

    def _afficher(self, detection, commande):
        import cv2
        image, masque, aire, p_hg, p_bd, centre = detection
        if aire:
            cv2.rectangle(image, p_hg, p_bd, (0, 255, 0), 2)
            cv2.circle(image, (int(centre[0]), int(centre[1])), 3, (0, 0, 255), -1)
        hauteur, largeur = image.shape[:2]
        for x in (largeur // 2 - MARGE_CENTRE, largeur // 2 + MARGE_CENTRE):
            cv2.line(image, (x, 0), (x, hauteur - 1), (255, 255, 0), 1)
        noms = {MSG_ARRETER: "ARRET", MSG_AVANCER: "AVANCE", MSG_PIVOTER_G: "GAUCHE", MSG_PIVOTER_D: "DROITE"}
        cv2.putText(image, f"Aire: {aire} px | {noms[commande]}", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        cv2.imshow(FENETRE_IMAGE, image)
        cv2.imshow(FENETRE_MASQUE, masque)
        touche = cv2.waitKeyEx(1)
        fermee = cv2.getWindowProperty(FENETRE_IMAGE, cv2.WND_PROP_VISIBLE) < 1
        if touche in (ord("x"), ord("X"), 27) or fermee:
            self._bloque = True
            self.desactiver()

    def dispatch_event(self, evenement):
        try:
            if evenement is not None:
                if evenement.type in (MSG_MODE_TELECOMMANDE, MSG_MODE_SUIVEUR):
                    if not self._filtre_mode.accepter(evenement):
                        evenement = None
            if evenement is not None:
                if evenement.type == MSG_MODE_TELECOMMANDE:
                    if self.actif or self._bloque:
                        self.desactiver()
                    self._bloque = False
                    return
                if evenement.type == MSG_MODE_SUIVEUR:
                    self._dernier_contact = self._horloge()
                    self._attente_contact = False
                    if not self.actif and not self._bloque:
                        self.envoyer_commande(MSG_ARRETER)
                        self.detecteur.demarrer()
                        self.actif = True
                        self._purger_modes_pendant_ouverture()
                        if self.actif:
                            self._attente_contact = True
                            self._debut_attente = self._horloge()
                        return

            if not self.actif:
                return
            if self._attente_contact:
                if self._horloge() - self._debut_attente >= DELAI_SECURITE_TELECOMMANDE:
                    self._bloque = True
                    self.desactiver()
                    print("Suiveur arrete : aucune confirmation apres ouverture camera.")
                return
            if self._horloge() - self._dernier_contact >= DELAI_SECURITE_TELECOMMANDE:
                self._bloque = True
                self.desactiver()
                print("Suiveur arrete : telecommande deconnecter.")
                return

            detection = self.detecteur.detecter(TEINTE_MIN, TEINTE_MAX)
            image, _, aire, _, _, centre = detection
            commande = choisir_commande(aire, centre[0], image.shape[1])
            self.envoyer_commande(commande)
            if self.affichage:
                self._afficher(detection, commande)
        except Exception as erreur:
            self._bloque = True
            self.desactiver()
            print(f"Suiveur arrete : {erreur}")

    def quitter(self):
        self.desactiver()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip-robot", default="127.0.0.1")
    parser.add_argument("--sans-affichage", action="store_true", help="Executer sur le Pi sans affichage.")
    options = parser.parse_args()
    suiveur = Suiveur(ip_robot=options.ip_robot, affichage=not options.sans_affichage)
    print(f"Suiveur en ecoute sur le port {APP_SUIVEUR}; camera en attente du mode 2.")
    try:
        suiveur.run()
    finally:
        fermer_client()


if __name__ == "__main__":
    main()
