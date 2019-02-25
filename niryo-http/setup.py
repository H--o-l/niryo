# -*- coding: utf-8 -*-
# !/usr/bin/env python

from setuptools import setup

from niryo_http import __author__, __version__

setup(
    name='niryo-http',
    version=__version__,
    author=__author__,
    url='TODO',
    # packages=['server'],
    # package_dir={'': 'niryo_http'},
    packages=['niryo_http'],
    entry_points={'console_scripts': ['niryo-http=niryo_http.api:start']},
    install_requires=[
        'Flask >=0.11.1',
        'Jinja2 >=2.8.1',
        'jsonschema >=2.6.0',
    ],
)
