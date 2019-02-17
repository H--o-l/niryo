# -*- coding: utf-8 -*-
#!/usr/bin/env python

import flask
import sys
import time
import threading
import urllib
from jsonschema import validate, ValidationError

# Niryo API
from niryo_one_python_api.niryo_one_api import *
import rospy

app = flask.Flask(__name__)
niryo = None

class InvalidUsage(Exception):
    def __init__(self, code, message=None, body=None):
        Exception.__init__(self)
        self.code = code
        self.message = message

@app.errorhandler(InvalidUsage)
def handle_invalid_usage(error):
    return error.message, error.code

@app.errorhandler(500)
def internal_error(error):
    body = {}
    if len(error.args) >= 2 and type(error.args[1]) == dict:
        body = error.args[1]

    body['Internal error'] = 'erreur inconnue'
    if len(error.args) >= 1 and type(error.args[0]) == str:
        body['Internal error'] = error.args[0]

    response = flask.jsonify(body)
    response.status_code = 500

    if flask.request.headers.get('Origin', None):
        add_cors_headers(response, flask.request.headers['Origin'])

    return response


class RospyHandler(threading.Thread):
    def __init__(self):
        super(RospyHandler, self).__init__()

    def run(self):
        global niryo
        rospy.init_node('niryo_one_python_api', anonymous=True, disable_signals=True)
        niryo = NiryoOne()
        rospy.spin()

def start():
    # Not a good idea finally ! 2 process are started and conflict.
    # app.debug = True

    rospy_handler = RospyHandler()
    rospy_handler.start()
    while not niryo:
        time.sleep(1)
    print '--- rospi init ok ---'
    app.run(host='0.0.0.0', port=6000, threaded=False)
    niryo.activate_learning_mode(True)
    print '--- closing ! ---'
    rospy.signal_shutdown('app is down')
    rospy_handler.join()
    print '--- rospi down ok ---'

def list_routes():
    routes = {}
    for r in app.url_map._rules:
        routes[r.rule] = {}
        routes[r.rule]['methods'] = [m for m in list(r.methods)
                                     if m != 'HEAD' and m != 'OPTIONS']
    routes.pop('/static/<path:filename>')
    return routes

@app.route('/', methods=['GET'])
def get_root():
    return flask.jsonify({'message': 'Hello !', 'routes': list_routes()})

@app.route('/joints', methods=['GET'])
def get_joints():
    return flask.jsonify(niryo.joints)

@app.route('/joints/<n>', methods=['GET'])
def get_joint(n):
    if int(n) < 0 or int(n) > 5:
        raise InvalidUsage(400, 'There is no joint n=' + n)
    return flask.jsonify(niryo.joints[int(n)])

@app.route('/hw', methods=['GET'])
def get_hw():
    return flask.jsonify(str(niryo.hw_status))

@app.route('/pose', methods=['GET'])
def get_pose():
    return flask.jsonify(str(niryo.pose))  # TODO make it json

@app.route('/learning', methods=['GET'])
def get_learning():
    return flask.jsonify(niryo.learning_mode_on)

@app.route('/io', methods=['GET'])
def get_io():
    return flask.jsonify(str(niryo.digital_io_state))  # TODO make it json

@app.route('/calibration/auto', methods=['POST'])
def post_calibration_auto():
    niryo.calibrate_auto()
    return 'OK'

@app.route('/calibration/manual', methods=['POST'])
def post_calibration_manual():
    niryo.calibrate_manual()
    return 'OK'

# curl -X POST 192.168.0.21:6000/start
@app.route('/start', methods=['POST'])
def post_start():
    niryo.activate_learning_mode(False)
    return 'OK'

# curl -X POST 192.168.0.21:6000/stop
@app.route('/stop', methods=['POST'])
def post_stop():
    niryo.activate_learning_mode(True)
    return 'OK'

# curl 192.168.0.21:6000/joints \
#   -d '[
#     -0.0634796041343917,
#     -0.0406281155201086,
#     -1.3344288795248074,
#     -3.04720524764194,
#     0.14744541520848095,
#     -0.05061454830783556
#   ]'
@app.route('/joints', methods=['POST'])
def post_joints():
    data = flask.request.get_json(force=True)
    schema = {
        'type': 'array',
        'items': {'type': "number"},
        'minItems': 6,
        'maxItems': 6,
    }
    try:
        validate(data, schema)
    except ValidationError as e:
        raise InvalidUsage(400, str(e))

    niryo.move_joints(data)
    return 'OK'

# curl 192.168.0.21:6000/joints/0 -d '-0.1'
@app.route('/joints/<n>', methods=['POST'])
def post_joint(n):
    if int(n) < 0 or int(n) > 5:
        raise InvalidUsage(400, 'There is no joint n=' + n)
    data = flask.request.get_json(force=True)
    schema = {'type': 'number'}
    try:
        validate(data, schema)
    except ValidationError as e:
        raise InvalidUsage(400, str(e))
    joints = niryo.joints
    joints[int(n)] = data
    niryo.move_joints(joints)
    return 'OK'
