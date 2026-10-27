"""Ignore les changements de mode perimes apres UDP."""


class FiltreMode:
    def __init__(self):
        self._generation = None
        self._mode = None

    def accepter(self, evenement):
        donnees = evenement.split()
        if donnees == [""]:
            return self._generation is None
        if len(donnees) != 1:
            raise ValueError("Une seule generation de mode est requise")
        generation = int(donnees[0])
        if generation < 0:
            raise ValueError("La generation de mode doit etre positive ou nulle")
        if self._generation is not None:
            if generation < self._generation:
                return False
            if generation == self._generation:
                return evenement.type == self._mode
        self._generation = generation
        self._mode = evenement.type
        return True
