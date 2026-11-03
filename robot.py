"""Mouvements du robot mobile a deux moteurs."""

import math

from param import (
    FACTEUR_MOTEUR_DROIT_PIVOT_D,
    FACTEUR_MOTEUR_GAUCHE_PIVOT_G,
    VITESSE_MAX,
    VITESSE_MIN,
    VITESSE_ROTATION_MIN,
    VITESSE_INITIALE,
)


class Robot:
    def __init__(self, moteur_gauche, moteur_droit):
        self._moteur_gauche = moteur_gauche
        self._moteur_droit = moteur_droit
        self.vitesse = VITESSE_INITIALE

    @staticmethod
    def limiter_vitesse(vitesse):
        vitesse = float(vitesse)
        if not math.isfinite(vitesse):
            raise ValueError("La vitesse doit etre finie")
        return max(VITESSE_MIN, min(VITESSE_MAX, vitesse))

    def avancer(self):
        puissance = self.limiter_vitesse(self.vitesse)
        if puissance == 0:
            self.arreter()
            return
        self._moteur_gauche.avancer(puissance)
        self._moteur_droit.avancer(puissance)

    def avancer_corrige(self, correction):
        """Petite correction de cap sans interrompre l'avance."""
        puissance = self.limiter_vitesse(self.vitesse)
        if puissance == 0:
            self.arreter()
            return
        correction = max(-min(0.05, puissance/2), min(min(0.05, puissance/2), correction))
        self._moteur_gauche.avancer(self.limiter_vitesse(puissance-correction))
        self._moteur_droit.avancer(self.limiter_vitesse(puissance+correction))

    def reculer(self):
        puissance = self.limiter_vitesse(self.vitesse)
        if puissance == 0:
            self.arreter()
            return
        self._moteur_gauche.reculer(puissance)
        self._moteur_droit.reculer(puissance)

    def pivoter_gauche(self):
        puissance = self.limiter_vitesse(self.vitesse)
        if puissance == 0:
            self.arreter()
            return
        puissance = max(VITESSE_ROTATION_MIN, puissance)
        puissance_gauche = self.limiter_vitesse(
            puissance * FACTEUR_MOTEUR_GAUCHE_PIVOT_G
        )
        self._moteur_gauche.reculer(puissance_gauche)
        self._moteur_droit.avancer(puissance)

    def pivoter_droite(self):
        puissance = self.limiter_vitesse(self.vitesse)
        if puissance == 0:
            self.arreter()
            return
        puissance = max(VITESSE_ROTATION_MIN, puissance)
        puissance_droite = self.limiter_vitesse(
            puissance * FACTEUR_MOTEUR_DROIT_PIVOT_D
        )
        self._moteur_gauche.avancer(puissance)
        self._moteur_droit.reculer(puissance_droite)

    def arreter(self):
        self._moteur_gauche.arreter()
        self._moteur_droit.arreter()

    def fermer(self):
        self.arreter()
        self._moteur_gauche.fermer()
        self._moteur_droit.fermer()
