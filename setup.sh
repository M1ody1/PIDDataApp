#!/bin/bash

# Create a virtual environment in the 'venv' directory
python -m venv venv

source venv/Scripts/activate

# Install the dependencies from requirements.txt
pip install -r requirements.txt