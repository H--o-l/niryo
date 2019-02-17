# -*- coding: utf-8 -*-
#!/usr/bin/env python

import flask
import sys
import time
import threading
import urllib

# Niryo API
from niryo_one_python_api.niryo_one_api import *
import rospy

app = flask.Flask(__name__)
niryo = None

class InvalidUsage(Exception):
    def __init__(self, code, message=None, body=None):
        Exception.__init__(self)
        self.code = code
        self.body = body if body is not None else {}
        if message is not None:
            self.body['message'] = message

    def to_flask_response(self):
        response = flask.jsonify(self.body)
        response.status_code = self.code
        return response

@app.errorhandler(InvalidUsage)
def handle_invalid_usage(error):
    return error.to_flask_response()

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
    response = flask.jsonify({'message': 'Hello !', 'routes': list_routes()})
    return response

@app.route('/joints', methods=['GET'])
def get_joints():
    response = flask.jsonify(niryo.joints)
    return response

@app.route('/hw', methods=['GET'])
def get_hw():
    response = flask.jsonify(str(niryo.hw_status))
    return response

@app.route('/pose', methods=['GET'])
def get_pose():
    response = flask.jsonify(str(niryo.pose))  # TODO make it json
    return response

@app.route('/learning', methods=['GET'])
def get_learning():
    response = flask.jsonify(niryo.learning_mode_on)
    return response

@app.route('/io', methods=['GET'])
def get_io():
    response = flask.jsonify(str(niryo.digital_io_state))  # TODO make it json
    return response

@app.route('/calibration/auto', methods=['POST'])
def post_calibration_auto():
    niryo.calibrate_auto()
    return 'OK'

@app.route('/calibration/manual', methods=['POST'])
def post_calibration_manual():
    niryo.calibrate_manual()
    return 'OK'

@app.route('/learning/<boolean>', methods=['POST'])
def post_learning(boolean):
    niryo.activate_learning_mode(boolean == 'true')
    return 'OK'

@app.route('/joints', methods=['POST'])
def post_joints():
    copy = niryo.joints[:]
    copy[2] = copy[2] + 0.1
    copy[0] = copy[0] - 0.1
    niryo.move_joints(copy)

    return 'OK'
