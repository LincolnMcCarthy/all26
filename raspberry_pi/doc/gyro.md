# Gyro

It's possible to use the Raspberry Pi to run a
gyroscope.  This was a project from 2024, when
there was no good gyro option.  We never used it
in a real robot.

We used the LSM6DS board from Adafruit, which
uses these libraries:

```
python3 -m pip install hidapi
python3 -m pip install adafruit-blinka
python3 -m pip install adafruit-circuitpython-lsm6ds
```