Ansible to deploy niryo-http on niryo robots
============================================

Using
-----

Change IPs in ``/inventories/niryo/hosts`` file, then:

.. code:: shell

  ansible-playbook -i inventories/niryo/hosts deploy.yml --ask-become-pass
