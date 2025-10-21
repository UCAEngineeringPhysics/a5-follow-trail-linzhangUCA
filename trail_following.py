from encoded_motor_driver import EncodedMotorDriver
from time import sleep
from machine import Pin
from math import pi


# SETUP
rm = EncodedMotorDriver((7, 9, 8), (16, 17))
lm = EncodedMotorDriver((15, 14, 13), (18, 19))
r = 0.025
i = 98.5
L = 0.12
CPR = 28
speed = 0.25

STBY = Pin(12, Pin.OUT)
STBY.off()


def distance_to_counts(targ_dist=0.0):
    targ_counts = targ_dist / (2 * pi * r) * i * CPR
    print(f"target encoder counts: {targ_counts}")
    return targ_counts


def angle_to_counts(targ_ang=0.0):
    targ_c = targ_ang * L / 2 / (2 * pi * r) * i * CPR
    print(f"target angular encoder counts: {targ_c}")
    return targ_c


# LOOP
sleep(1)
STBY.on()

# ----CP1----#
lin_tc1 = distance_to_counts(0.75)
while rm.encoder_counts <= lin_tc1 and lm.encoder_counts <= lin_tc1:
    print(rm.encoder_counts, lm.encoder_counts)
    if rm.encoder_counts - lm.encoder_counts > CPR:
        rm.stop()
        lm.forward(speed)
    elif lm.encoder_counts - rm.encoder_counts > CPR:
        rm.forward(speed)
        lm.stop()
    else:
        rm.forward(speed)
        lm.forward(speed)
    sleep(0.01)
rm.stop()
lm.stop()
sleep(3)
rm.reset_encoder_counts()
lm.reset_encoder_counts()

ang_tc1 = angle_to_counts(pi / 2)
while rm.encoder_counts <= ang_tc1 and lm.encoder_counts >= -ang_tc1:  # ccw
    print(rm.encoder_counts, lm.encoder_counts)
    if rm.encoder_counts + lm.encoder_counts > CPR:
        rm.stop()
        lm.backward(speed)
    elif lm.encoder_counts + rm.encoder_counts < -CPR:
        rm.forward(speed)
        lm.stop()
    else:
        rm.forward(speed)
        lm.backward(speed)
    sleep(0.01)
rm.stop()
lm.stop()
sleep(1)
rm.reset_encoder_counts()
lm.reset_encoder_counts()

# ----CP2----#
lin_tc2 = distance_to_counts(0.5)
while rm.encoder_counts <= lin_tc2 and lm.encoder_counts <= lin_tc2:
    print(rm.encoder_counts, lm.encoder_counts)
    if rm.encoder_counts - lm.encoder_counts > CPR:
        rm.stop()
        lm.forward(speed)
    elif lm.encoder_counts - rm.encoder_counts > CPR:
        rm.forward(speed)
        lm.stop()
    else:
        rm.forward(speed)
        lm.forward(speed)
    sleep(0.01)
rm.stop()
lm.stop()
sleep(3)
rm.reset_encoder_counts()
lm.reset_encoder_counts()

ang_tc2 = angle_to_counts(3 * pi / 2)
while rm.encoder_counts >= -ang_tc2 and lm.encoder_counts <= ang_tc2:  # cw
    print(rm.encoder_counts, lm.encoder_counts)
    if rm.encoder_counts + lm.encoder_counts > CPR:
        rm.backward()
        lm.stop()
    elif lm.encoder_counts + rm.encoder_counts < -CPR:
        rm.stop()
        lm.forward(speed)
    else:
        rm.backward(speed)
        lm.forward(speed)
    sleep(0.01)
rm.stop()
lm.stop()
sleep(1)
rm.reset_encoder_counts()
lm.reset_encoder_counts()

# TERMINATION
STBY.off()
