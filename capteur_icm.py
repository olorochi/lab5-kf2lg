"""Adaptation du pilote Pimoroni."""

import struct
import time


class CapteurICM:
    def __init__(self, capteur):
        self.capteur = capteur
        capteur.set_accelerometer_full_scale(2)
        capteur.set_accelerometer_sample_rate(200)
        capteur.set_accelerometer_low_pass(enabled=True, mode=4)
        capteur.set_gyro_sample_rate(200)
        capteur.set_gyro_full_scale(500)
        capteur.set_gyro_low_pass(enabled=True, mode=4)
        capteur.mag_write(0x31, 0x00)
        time.sleep(0.01)
        capteur.mag_write(0x31, 0x08)
        time.sleep(0.02)

    def read_accelerometer_gyro_data(self):
        return self.capteur.read_accelerometer_gyro_data()

    def read_magnetometer_data(self):
        donnees = self.capteur.mag_read_bytes(0x11, 8)
        if len(donnees) != 8 or donnees[7] & 0x08:
            raise ValueError("Lecture magnetique absente ou saturee")
        return tuple(v*0.15 for v in struct.unpack("<hhh", bytes(donnees[:6])))
