__Table of Contents__

[[_TOC_]]

# 1. Introduction

__Modbus Scale Client__ provides the client-side capabilities required to
communicate with the Mazzer and Rancilio weigh scales of project
UC-02-2024 _Coffee Cart Modifications_.

# 2. Dependencies

__Modbus Scale Client__ is part of the UC-02-2024 _Coffee Cart Modifications_
software suite. Scale software has been split along hardware component lines,
with the software for each hardware component residing in a separate GitLab
repository.

1. [modbus_scale_broker][modbus_scale_broker_gitlab]. The Arduino sketch that
must be downloaded to the Uno.

2. [modbus_scale_server][modbus_scale_server_gitlab]. The Modbus Ethernet
server daemon that runs on the Raspberry Pi.

3. [modbus_scale_client][modbus_scale_client_gitlab]. The Modbus Ethernet
client that queries the server.

4. [modbus_scale_ui][modbus_scale_ui_gitlab]. A [Textual][textual] UI (_User
Interface_) that displays the output of the Mazzer and Rancilio scales, and
which can be used to tare (zero) either scale.

Because these repositories are designed to be deployed together as part of a
comprehensive weigh scale solution, both hardware and software dependencies
exist between them. These dependencies must be borne in mind when deciding
whether or not to employ __Modbus Scale Client__ in isolation.

Python packages are also available for the following software components.

 - [modbus_scale_server][modbus_scale_server_pypi]
 - [modbus_scale_client][modbus_scale_client_pypi]
 - [modbus_scale_ui][modbus_scale_ui_pypi]

These packages may be installed using [pip][pip]. Note however that the caveat
regarding hardware and software dependencies still applies.

# 3. Installation

Two installation methods are available, with the most appropriate depending on
whether the intent is to use the code base as-is, or to modify it.

## 3.1. PyPI Package

Those who wish to use the code base as-is should install the Python package via
pip.

Whilst it is not strictly necessary to create a venv (_virtual environment_) in
order to deploy the package, doing so provides a Python environment that is
completely isolated from the system-wide Python install. The practical upshot
of this is that the venv can be torn-down and recreated multiple times without
issue.

    $ python -m venv ~/venv

Next, activate the venv and install the package. Note that once activated, the
name of the venv will be prepended to the terminal prompt.

    $ source ~/venv/bin/activate
    (venv) $ python -m pip install modbus-scale-client

## 3.2. GitLab Repository

Those who wish to modify the code base should clone the GitLab repository
instead. Again, whilst not strictly necessary to create a venv in order to
modify the code base, doing so is still recommended.

    $ python -m venv ~/venv
    $ source ~/venv/bin/activate
    (venv) $ cd ~/venv
    (venv) $ git clone https://gitlab.com/uc_mech_wing/robotics_control_lab/uc-02-2024/modbus_scale_client.git

Irrespective of whether or not a venv has been created, the _requirements.txt_
file may be used to ensure that the correct module dependencies are installed.

    (venv) $ cd ~/venv/modbus_scale_client
    (venv) $ python -m pip install -r ./requirements.txt

# 4. Verification

In order to verify that __Modbus Scale Client__ has been installed correctly,
it is advisable to create a minimal working example.

    $ touch ~/venv/example.py

If installed from the Python package, add the following.

    import time
    
    from modbus_scale_client import modbus_scale_client
    
    client = modbus_scale_client.ModbusScaleClient(host = "<host>")
    
    print("[INFO] Starting reading...")
    
    while True:
        try:
            weight = client.read()
            weight = "{:8.1f}".format(weight)
            
            if client.server_exists() == True and \
                client.broker_exists() == True:
                
                print("[REAL] Scale weight = %sg" % weight)
            
            elif client.server_exists() == True and \
                client.broker_exists() == False:
                
                print("[ERROR] No broker detected")
            
            else:
                print("[FAKE] Scale weight = %sg" % weight)
            
            time.sleep(1)
        
        except (KeyboardInterrupt, SystemExit):
            print("[INFO] Stopping reading...")
            sys.exit()

If installed from the GitLab repository, replace the import statement with the
following.

    from modbus_scale_client.src.modbus_scale_client import modbus_scale_client

In either case, be sure to replace `<host>` with the IPv4 (_Internet Protocol
version 4_) address of your physical Raspberry Pi. All going well, running the
example code will produce output similar to the following.

    (venv) $ python ~/venv/example.py
    [INFO] Starting reading... 
    [REAL] Scale weight =    123.4g
    [REAL] Scale weight =    123.4g
    [REAL] Scale weight =    123.4g
    ...
    [REAL] Scale weight =    123.4g
    [INFO] Stopping reading...

Note that verification assumes that all of the requisite hardware and software
dependencies have been met.

# 5. Further Information 

For further information about _Coffee Cart Modifications_ please refer to the
project [UC-02-2024][uc-02-2024_gitlab] group README.

# 6. Documentation

Code has been documented using [Doxygen][doxygen].

# 7. License

__Modbus Scale Client__ is released under the [GNU General Public License][gpl].

# 8. Authors

Code by Rodney Elliott, <rodney.elliott@canterbury.ac.nz>

[modbus_scale_broker_gitlab]: https://gitlab.com/uc_mech_wing/robotics_control_lab/uc-02-2024/modbus_scale_broker
[modbus_scale_server_gitlab]: https://gitlab.com/uc_mech_wing/robotics_control_lab/uc-02-2024/modbus_scale_server
[modbus_scale_client_gitlab]: https://gitlab.com/uc_mech_wing/robotics_control_lab/uc-02-2024/modbus_scale_client
[modbus_scale_ui_gitlab]: https://gitlab.com/uc_mech_wing/robotics_control_lab/uc-02-2024/modbus_scale_ui
[textual]: https://textual.textualize.io/
[modbus_scale_server_pypi]: https://pypi.org/project/modbus-scale-server/
[modbus_scale_client_pypi]: https://pypi.org/project/modbus-scale-client/
[modbus_scale_ui_pypi]: https://pypi.org/project/modbus-scale-ui/
[pip]: https://pypi.org/project/pip/
[uc-02-2024_gitlab]: https://gitlab.com/uc_mech_wing/robotics_control_lab/uc-02-2024
[doxygen]: https://www.doxygen.nl
[gpl]: https://www.gnu.org/licenses/gpl-3.0.html

