# -*- coding: utf-8 -*-
# !/usr/bin/env python3

import keyboard
import requests

endpoint = 'http://192.168.0.21:6000'

ctrl = False
alt = False
gripper_open = False

step = 0.2

mapping = {
    'none': {
        'h': 0,
        'v': 1,
    },
    'ctrl': {
        'h': 3,
        'v': 2,
    },
    'alt': {
        'h': 5,
        'v': 4,
    },
}

def ctrl_change(state):
    global ctrl
    ctrl = state

def alt_change(state):
    global alt
    alt = state

# def get_joint(n):
#     return requests.get(endpoint + '/joints/' + str(n)).json()

def move_joint(n, value):
    return requests.post(endpoint + '/joints_abs/' + str(n), str(value))

def on_press(toto):
    print(toto)

def up():
    joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['v']
    move_joint(joint_id, + step)

def down():
    joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['v']
    move_joint(joint_id, - step)

def left():
    joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['h']
    move_joint(joint_id, + step)

def right():
    joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['h']
    move_joint(joint_id, - step)

def gripper_toogle():
    global gripper_open
    if gripper_open:
        requests.post(endpoint + '/gripper/3/close')
        gripper_open = False
    else:
        requests.post(endpoint + '/gripper/3/open')
        gripper_open = True


keyboard.on_press_key('ctrl', lambda _: ctrl_change(True))
keyboard.on_release_key('ctrl', lambda _: ctrl_change(False))

keyboard.on_press_key('shift', lambda _: alt_change(True))
keyboard.on_release_key('shift', lambda _: alt_change(False))

try:
    while True:
        key = keyboard.read_key()
        if key == 'up':
            up()
        elif key == 'down':
            down()
        elif key == 'left':
            left()
        elif key == 'right':
            right()
        elif key == 'enter':
            gripper_toogle()
        # time.sleep(0.3)
except KeyboardInterrupt:
    print('stoping')
    requests.post(endpoint + '/stop')
