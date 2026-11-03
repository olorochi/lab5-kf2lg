"""Caps circulaires et comptage de tours."""

import math


def ecart_cap(cible, cap):
    return (cible - cap + 180.0) % 360.0 - 180.0


def moyenne_caps(caps):
    caps = tuple(caps)
    if not caps:
        raise ValueError("Il faut au moins un cap")
    sinus = sum(math.sin(math.radians(c)) for c in caps)
    cosinus = sum(math.cos(math.radians(c)) for c in caps)
    if math.hypot(sinus, cosinus) < 1e-6:
        raise ValueError("Caps disperses : orientation non definie")
    return math.degrees(math.atan2(sinus, cosinus)) % 360.0


class CapDeroule:
    def __init__(self, cap, progression=0.0):
        if not math.isfinite(cap) or not math.isfinite(progression):
            raise ValueError("Cap et progression doivent etre finis")
        self.cap = cap % 360.0
        self.progression = progression

    def actualiser(self, cap):
        if not math.isfinite(cap):
            raise ValueError("Le cap doit etre fini")
        self.progression += ecart_cap(cap, self.cap)
        self.cap = cap % 360.0
        return self.progression

    @classmethod
    def depuis_reference(cls, reference, cap, angle_approximatif):
        """Le gyro choisit le nombre de tours; le compas fixe la fraction."""
        fraction = ecart_cap(cap, reference)
        tours = round((angle_approximatif - fraction) / 360.0)
        return cls(cap, fraction + tours * 360.0)
