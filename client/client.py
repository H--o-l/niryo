# -*- coding: utf-8 -*-
# !/usr/bin/env python3

import keyboard
import requests
import json

endpoint = 'http://10.10.10.10:6000'

ctrl = False
alt = False
axis = None
y = False
z = False
roll = False
pitch = False
yaw = False
gripper_open = False
positions_list = dict()

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

def axis_change(name, state):
    global axis
    if state:
        axis = name
    else:
        axis = None

def learning_toogle():
    learning = requests.get(endpoint + '/learning').json() is True
    if learning:
        requests_post_and_print(endpoint + '/start')
    else:
        requests_post_and_print(endpoint + '/stop')


keyboard.on_press_key('ctrl', lambda _: ctrl_change(True))
keyboard.on_release_key('ctrl', lambda _: ctrl_change(False))

keyboard.on_press_key('alt', lambda _: alt_change(True))
keyboard.on_release_key('alt', lambda _: alt_change(False))

keyboard.on_press_key('x', lambda _: axis_change('x', True))
keyboard.on_release_key('x', lambda _: axis_change('x', False))

keyboard.on_press_key('y', lambda _: axis_change('y', True))
keyboard.on_release_key('y', lambda _: axis_change('y', False))

keyboard.on_press_key('z', lambda _: axis_change('z', True))
keyboard.on_release_key('z', lambda _: axis_change('z', False))

keyboard.on_press_key('r', lambda _: axis_change('roll', True))
keyboard.on_release_key('r', lambda _: axis_change('roll', False))

keyboard.on_press_key('p', lambda _: axis_change('pitch', True))
keyboard.on_release_key('p', lambda _: axis_change('pitch', False))

keyboard.on_press_key('i', lambda _: axis_change('yaw', True))
keyboard.on_release_key('i', lambda _: axis_change('yaw', False))

keyboard.on_press_key('n', lambda _: requests_post_and_print(
    endpoint + '/pose/initial'))

keyboard.on_press_key('c', lambda _: requests_post_and_print(
    endpoint + '/calibration/manual'))

keyboard.on_press_key('l', lambda _: learning_toogle())


# def get_joint(n):
#     return requests.get(endpoint + '/joints/' + str(n)).json()

def requests_post_and_print(url, data=None):
    res = requests.post(url, data)
    print()
    print(res.status_code)
    print(res.text)

def move_joint(n, value):
    requests_post_and_print(endpoint + '/shift_joint/' + str(n), str(value))

def move_axis(axis, value):
    requests_post_and_print(endpoint + '/shift_pose/' + axis, str(value))

def get_pose():
    return dict(requests.get(endpoint + '/pose').json())

def move_pose(pose):
    requests_post_and_print(endpoint + '/pose', json.dumps(pose))

def up():
    global axis
    if axis:
        move_axis(axis, 0.05)
    else:
        joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['v']
        move_joint(joint_id, + step)

def down():
    global axis
    if axis:
        move_axis(axis, - 0.05)
    else:
        joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['v']
        move_joint(joint_id, - step)

def left():
    if not axis:
        joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['h']
        move_joint(joint_id, + step)
    return None

def right():
    if not axis:
        joint_id = mapping['ctrl' if ctrl else 'alt' if alt else 'none']['h']
        move_joint(joint_id, - step)
    return None

def gripper_toogle():
    global gripper_open
    if gripper_open:
        gripper_open = False
        requests_post_and_print(endpoint + '/gripper/2/close')
    else:
        gripper_open = True
        requests_post_and_print(endpoint + '/gripper/2/open')

def number(n):
    global positions_list
    if ctrl:
        positions_list[n] = get_pose()
        print(json.dumps(positions_list, indent=4, sort_keys=True))
    elif positions_list.get(n, None) is not None:
        print('Moving to ' + n)
        move_pose(positions_list[n])
    else:
        print('position unknow: ' + n)
    return None


try:
    while True:
        key = keyboard.read_event()
        if key.event_type == 'up':
            continue

        if key.name == 'up':
            up()
        elif key.name == 'down':
            down()
        elif key.name == 'left':
            left()
        elif key.name == 'right':
            right()
        elif key.name == 'enter':
            gripper_toogle()
        elif key.scan_code == 2:
            number('1')
        elif key.scan_code == 3:
            number('2')
        elif key.scan_code == 4:
            number('3')
        elif key.scan_code == 5:
            number('4')
        elif key.scan_code == 6:
            number('5')

        # time.sleep(0.3)
except KeyboardInterrupt:
    print('stoping')
    requests.post(endpoint + '/stop')
