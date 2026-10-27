#!/usr/bin/env python3

"""Menu clavier : telecommande manuelle ou poursuite avec la camera du Pi."""

import argparse
import time

from ev_app_client_api import fermer_client, gen_ev_externe
from param import (
    APP_CTRL_ROBOT, APP_SUIVEUR, INTERVALLE_RENOUVELLEMENT_COMMANDE,
    IP_CTRL_ROBOT, MSG_ARRETER, MSG_AVANCER, MSG_MODE_SUIVEUR,
    MSG_MODE_TELECOMMANDE, MSG_PIVOTER_D, MSG_PIVOTER_G, MSG_RECULER,
    MSG_VITESSE, PAS_VITESSE, VITESSE_INITIALE, VITESSE_MAX, VITESSE_MIN,
)

NOM_FENETRE = "Robot - menu et telecommande"
LARGEUR_FENETRE = 560
HAUTEUR_FENETRE = 440
NOMS_COMMANDES = {
    MSG_ARRETER: "ARRET", MSG_AVANCER: "AVANCE", MSG_RECULER: "RECULE",
    MSG_PIVOTER_G: "GAUCHE", MSG_PIVOTER_D: "DROITE",
}


class TelCmd:
    COMMANDES_MOUVEMENT = {
        ord("q"): MSG_PIVOTER_G, ord("w"): MSG_AVANCER,
        ord("e"): MSG_PIVOTER_D, ord("s"): MSG_RECULER,
        ord(" "): MSG_ARRETER,
    }

    def __init__(self, ip_robot, envoyer=gen_ev_externe, horloge=time.monotonic):
        self.ip_robot = ip_robot
        self.vitesse = VITESSE_INITIALE
        self.mode = None
        self.derniere_commande = MSG_ARRETER
        self._envoyer = envoyer
        self._horloge = horloge
        self._dernier_envoi = float("-inf")
        self._generation_mode = time.time_ns()

    def _envoyer_mode(self, mode):
        self._envoyer(self.ip_robot, APP_CTRL_ROBOT, mode, self._generation_mode)
        self._envoyer(self.ip_robot, APP_SUIVEUR, mode, self._generation_mode)

    def envoyer_mouvement(self, type_message):
        self._envoyer(self.ip_robot, APP_CTRL_ROBOT, type_message)
        self.derniere_commande = type_message

    def changer_mode(self, mode):
        if mode not in (None, MSG_MODE_TELECOMMANDE, MSG_MODE_SUIVEUR):
            raise ValueError("Mode inconnu")
        self._generation_mode = max(self._generation_mode + 1, time.time_ns())
        self._envoyer_mode(MSG_MODE_TELECOMMANDE)
        self.envoyer_mouvement(MSG_ARRETER)
        self.mode = mode
        self._envoyer(self.ip_robot, APP_CTRL_ROBOT, MSG_VITESSE, self.vitesse)
        if mode == MSG_MODE_SUIVEUR:
            self._generation_mode += 1
            self._envoyer_mode(MSG_MODE_SUIVEUR)
        self._dernier_envoi = float("-inf")

    def arreter(self):
        self.changer_mode(None)

    def maintenir_connexion(self):
        maintenant = self._horloge()
        if maintenant - self._dernier_envoi < INTERVALLE_RENOUVELLEMENT_COMMANDE:
            return
        self._dernier_envoi = maintenant
        self._envoyer_mode(self.mode or MSG_MODE_TELECOMMANDE)
        if self.mode != MSG_MODE_SUIVEUR:
            self.envoyer_mouvement(self.derniere_commande)

    def modifier_vitesse(self, variation):
        self.vitesse = round(max(VITESSE_MIN,
                                 min(VITESSE_MAX, self.vitesse + variation)), 2)
        self._envoyer(self.ip_robot, APP_CTRL_ROBOT, MSG_VITESSE, self.vitesse)

    def traiter_touche(self, touche):
        if touche == -1:
            return True
        if ord("A") <= touche <= ord("Z"):
            touche = ord(chr(touche).lower())
        if touche in (ord("x"), 27):
            return False
        if touche == ord("1"):
            self.changer_mode(MSG_MODE_TELECOMMANDE)
        elif touche == ord("2"):
            self.changer_mode(MSG_MODE_SUIVEUR)
        elif touche == ord("m"):
            self.arreter()
        elif touche == ord("."):
            self.modifier_vitesse(PAS_VITESSE)
        elif touche == ord(","):
            self.modifier_vitesse(-PAS_VITESSE)
        elif touche == ord(" ") and self.mode == MSG_MODE_SUIVEUR:
            self.arreter()
        elif self.mode == MSG_MODE_TELECOMMANDE:
            commande = self.COMMANDES_MOUVEMENT.get(touche)
            if commande is not None:
                self.envoyer_mouvement(commande)
        return True

    def executer(self, lire_touche, actualiser_affichage=None):
        try:
            self.arreter()
            while self.traiter_touche(lire_touche()):
                self.maintenir_connexion()
                if actualiser_affichage is not None:
                    actualiser_affichage()
        except KeyboardInterrupt:
            pass
        finally:
            self.arreter()


def afficher_commandes(telecommande, cv2, np):
    image = np.full((HAUTEUR_FENETRE, LARGEUR_FENETRE, 3),
                    (32, 32, 32), dtype=np.uint8)
    police = cv2.FONT_HERSHEY_SIMPLEX

    def texte(libelle, y, couleur=(235, 235, 235), taille=0.6):
        cv2.putText(image, libelle, (30, y), police, taille, couleur, 1, cv2.LINE_AA)

    texte("ROBOT MOBILE", 38, (80, 210, 255), 0.85)
    modes = {None: "MENU", MSG_MODE_TELECOMMANDE: "TELECOMMANDE",
             MSG_MODE_SUIVEUR: "SUIVEUR DE BALLE JAUNE"}
    texte(modes[telecommande.mode], 76, (100, 230, 120))
    texte("1  Telecommande", 120)
    texte("2  Suiveur (camera du Pi)", 156)
    if telecommande.mode == MSG_MODE_TELECOMMANDE:
        texte("Q  Gauche       W  Avance       E  Droite", 204, taille=0.53)
        texte("S  Recule       ESPACE  Arret", 238, taille=0.53)
        texte(f"Mouvement: {NOMS_COMMANDES[telecommande.derniere_commande]}", 278)
    elif telecommande.mode == MSG_MODE_SUIVEUR:
        texte("ESPACE  Arret", 214)
    texte(f"Puissance PWM: {telecommande.vitesse:.0%}", 320, (100, 230, 120))
    texte(",  Diminuer       .  Augmenter", 352, taille=0.53)
    texte("M  Menu / arret       X  Quitter", 410, taille=0.53)
    cv2.imshow(NOM_FENETRE, image)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ip_robot", nargs="?", default=IP_CTRL_ROBOT)
    options = parser.parse_args()
    try:
        import cv2
        import numpy as np
    except ImportError as erreur:
        raise SystemExit("OpenCV et NumPy sont necessaires a la telecommande.") from erreur
    telecommande = TelCmd(options.ip_robot)
    afficher_commandes(telecommande, cv2, np)
    print(f"Robot : {options.ip_robot}, controleur {APP_CTRL_ROBOT}, suiveur {APP_SUIVEUR}.")

    def lire_touche():
        touche = cv2.waitKeyEx(30)
        if cv2.getWindowProperty(NOM_FENETRE, cv2.WND_PROP_VISIBLE) < 1:
            return ord("x")
        return touche

    try:
        telecommande.executer(lire_touche,
                              lambda: afficher_commandes(telecommande, cv2, np))
    finally:
        cv2.destroyAllWindows()
        fermer_client()


if __name__ == "__main__":
    main()
