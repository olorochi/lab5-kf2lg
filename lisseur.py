class Lisseur:
    def __init__(self, window_size):
        self.recorded = [None for _ in range(window_size)]
        self.__id = 0

    @staticmethod
    def __lisser(values):
        length = len(values)
        if (length == 0):
            return None

        return sum(values) / length

    def __values(self):
        return [v for v in self.recorded if v is not None]

    def lisser(self):
        return self.__lisser(self.__values())

    def lisser_min_max(self):
        values = self.__values()
        maximum = max(values)
        minimum = min(values)
        values = [v for v in values if v != maximum and v != minimum]
        return self.__lisser(values)

    def add(self, v):
        self.recorded[self.__id % len(self.recorded)] = v
        self.__id += 1


"""Test"""
if __name__ == "__main__":
    l = Lisseur(3)
    assert l.lisser() == None
    l.add(5)
    assert l.lisser() == 5
    l.add(15)
    assert l.lisser() == 10
    assert l.lisser_min_max() == None
    l.add(10)
    assert l.lisser() == 10
    assert l.lisser_min_max() == 10
    l.add(5)
    assert l.lisser() == 10
    assert l.lisser_min_max() == 10
