# -*- coding: utf-8 -*-
#!/usr/bin/env python

import flask


app = flask.Flask(__name__)


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


class ServerError(Exception):
    def __init__(self, code, message):
        Exception.__init__(self)
        self.code = code
        self.body = {'message': message}

    def to_flask_response(self):
        response = flask.jsonify(self.body)
        response.status_code = self.code
        return response


def start():
    app.run(host='0.0.0.0', port=6000, threaded=True)


@app.route('/', methods=['GET'])
def auth_demo_route():
    # data = flask.request.get_json(force=True)

    response = flask.jsonify({'ok': True, 'message': 'coucou !'})
    return response

