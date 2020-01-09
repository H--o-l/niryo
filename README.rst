Niryo
=====

Info
----

Learning support for Python and Niryo.

See sub-folders.


Resources
----------

`Niryo Python API`_

.. _Niryo Python API: https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_python_api


Notes
-----

Add Python and pip to windows 10 path in PowerShell:

.. code:: powershell

 $env:PATH = $env:PATH + ";C:\Users\H__o_l\AppData\Local\Programs\Python\Python37-32\;C:\Users\H__o_l\AppData\Local\Programs\Python\Python37-32\Scripts\"


In PowerShell, ``curl`` is in fact Windows ``Invoke-WebRequest``, usage:

.. code:: powershell

 (curl 192.168.0.21:6000/joints).content | python -m json.tool


Mount niryo rpi folder with sshfs:

.. code:: shell

 sshfs niryo@<niryo-IP>:/home/niryo mount


Website
-------

https://hoel.dev/iut
