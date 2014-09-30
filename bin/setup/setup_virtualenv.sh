#!/bin/bash

exit 1  # This is just a sketch. Get it working before removing this...

# Download virtualenv.py (or maybe just have it in helpers?)
# curl -L -o virtualenv.py https://raw.github.com/pypa/virtualenv/master/virtualenv.py

# Call virtualenv.py with --no-setuptools to create a new virtualenv
python virtualenv.py -p /usr/bin/python2.7 --no-setuptools venv2

# Activate the new virtualenv
# source $venv/bin/activate (?)

# Install pip
easy_install pip

# Anything else?

# Deactivate the virtualenv
deactivate


