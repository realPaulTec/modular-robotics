#!/bin/sh

# Check if the virtual environment folder exists
if [ ! -d ".venv" ]; then
    # Create the virtual environment
    python3 -m venv .venv
fi

# Activate the virtual environment
. .venv/bin/activate

# Check if the requirements are already installed
pip freeze > installed.txt
if ! diff -q requirements.txt installed.txt > /dev/null; then
    # Install requirements
    pip install -r requirements.txt
fi
rm installed.txt

# Start your Python program
python src/tracking_lidar.py
