```bash
cd /home/pi/lab5-KF2CLG-main
sudo apt install python3-venv python3-gpiozero python3-lgpio
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install gpiozero icm20948 smbus2
```

## Lancement

Terminal 1 :

```bash
cd /home/pi/lab5-KF2CLG-main
source .venv/bin/activate
python3 ctrl_robot.py --journal mesures_imu.csv
```

Terminal 2 (possible pendant ou apres la calibration) :

```bash
cd /home/pi/lab5-KF2CLG-main
source .venv/bin/activate
python3 patrouilleur.py
```