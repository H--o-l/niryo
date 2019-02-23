# -*- coding: utf-8 -*-
#!/usr/bin/env python

import flask
import sys
import time
import threading
import urllib
from jsonschema import validate, ValidationError
import logging
import signal

# Niryo API
from niryo_one_python_api.niryo_one_api import *
import rospy

app = flask.Flask(__name__)
app.logger.setLevel(logging.INFO)

niryo = None
rospy_handler = None
last_tool = TOOL_NONE
supported_tools = ['3'] # Only grip 3 for now, otherwise /!\ Rospi deadlock

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

@app.after_request
def after_request(response):
    if response.status_code == 200:
        app.logger.info('%s %s %s %s',
                        flask.request.remote_addr,
                        flask.request.method,
                        flask.request.full_path,
                        response.status)
    else:
        app.logger.info('%s %s %s %s - %s',
                        flask.request.remote_addr,
                        flask.request.method,
                        flask.request.full_path,
                        response.status,
                        response.get_data())
    return response

class RospyHandler(threading.Thread):
    def __init__(self):
        super(RospyHandler, self).__init__()

    def run(self):
        global niryo
        rospy.init_node('niryo_one_python_api', anonymous=True, disable_signals=True)
        niryo = NiryoOne()
        rospy.spin()

def stop(*argv):
    global rospy_handler
    try:
        niryo.activate_learning_mode(True)
    except:
        pass
    rospy.signal_shutdown('Server going down')
    rospy_handler.join();
    print '--- rospy closing ok ---'
    raise RuntimeError('Server going down')

def exit_with_error():
    try:
        rospy.signal_shutdown('Server going down')
    except:
        pass
    app.logger.exception('niryo-http: kill invoqued')
    sys.exit(1)

def start():
    try:
        global rospy_handler
        rospy_handler = RospyHandler()
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        rospy_handler.start()
        timeout = 0
        while not niryo and rospy_handler.isAlive():
            if timeout > 10:
                exit_with_error()
            time.sleep(1)
            timeout = timeout + 1
        if not rospy_handler.isAlive():
            exit_with_error()
        print '--- rospi init ok ---'
        app.run(host='0.0.0.0', port=6000, threaded=False, debug=False, use_reloader=False)
    except RuntimeError, msg:
        print('coucou !')
        if str(msg) == "Server going down":
            print '--- app closing ok ---'
        else:
            raise RuntimeError(msg)
    except:
        exit_with_error()

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
        raise InvalidUsage(400, e.message)

    try:
        niryo.move_joints(data)
    except NiryoOneException as e:
        raise InvalidUsage(400, str(e))

    return 'OK'

# curl 192.168.0.21:6000/joints/0 -d '-0.1'
# In PowerShell:
# curl 192.168.0.21:6000/joints/0 -Body '-0.1' -Method POST
@app.route('/joints/<n>', methods=['POST'])
def post_joint(n):
    if int(n) < 0 or int(n) > 5:
        raise InvalidUsage(400, 'There is no joint n=' + n)
    data = flask.request.get_json(force=True)
    schema = {'type': 'number'}
    try:
        validate(data, schema)
    except ValidationError as e:
        raise InvalidUsage(400, e.message)
    joints = niryo.joints
    joints[int(n)] = data

    try:
        niryo.move_joints(joints)
    except NiryoOneException as e:
        raise InvalidUsage(400, str(e))

    return 'OK'

@app.route('/gripper/unset', methods=['POST'])
def unset_gripper():
    niryo.change_tool(0)
    print 'Tool unset'
    return 'OK'

# curl -X POST 192.168.0.21:6000/gripper/3/open
@app.route('/gripper/<n>/open', methods=['POST'])
def open_gripper(n):
    global last_tool
    if n not in supported_tools:
        raise InvalidUsage(400, 'Unsupported tool: ' + n + '\n'
                                'Supported tools are: ' + str(supported_tools))
    if n == '1':
        tool = TOOL_GRIPPER_1_ID
    elif n == '2':
        tool = TOOL_GRIPPER_2_ID
    elif n == '3':
        tool = TOOL_GRIPPER_3_ID
    else:
        raise InvalidUsage(400, 'Should not be here, tool not supported: ' + n)
    if tool != last_tool:
        last_tool = tool
        niryo.change_tool(tool)
        app.logger.info('Tool changed for ' + n)
    learning_before = niryo.learning_mode_on
    niryo.open_gripper(tool, 300)
    if learning_before == True:
        niryo.activate_learning_mode(True)
    return 'OK'

# curl -X POST 192.168.0.21:6000/gripper/3/close
@app.route('/gripper/<n>/close', methods=['POST'])
def close_gripper(n):
    global last_tool
    if n not in supported_tools:
        raise InvalidUsage(400, 'Unsupported tool: ' + n + '\n'
                                'Supported tools are: ' + str(supported_tools))
    if n == '1':
        tool = TOOL_GRIPPER_1_ID
    elif n == '2':
        tool = TOOL_GRIPPER_2_ID
    elif n == '3':
        tool = TOOL_GRIPPER_3_ID
    else:
        raise InvalidUsage(400, 'Should not be here, tool not supported: ' + n)
    if tool != last_tool:
        last_tool = tool
        niryo.change_tool(tool)
        app.logger.info('Tool changed for ' + n)
    learning_before = niryo.learning_mode_on
    niryo.close_gripper(tool, 300)
    if learning_before == True:
        niryo.activate_learning_mode(True)
    return 'OK'
