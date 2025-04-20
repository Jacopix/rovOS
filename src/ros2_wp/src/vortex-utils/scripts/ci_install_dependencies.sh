#!/bin/bash

# Script to install dependencies for H264Decoder
# This script installs GStreamer and PyGObject dependencies required for running the tests

set -e  # Exit on error

### GStreamer Installation ###
echo "Installing GStreamer and related plugins..."
sudo apt update
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav python3-gi \
    python3-gst-1.0

echo "GStreamer installation completed."

echo "If you experience display-related issues with the GUI, try running:"
echo "export QT_QPA_PLATFORM=xcb"

### PyGObject Installation ###
echo "Installing PyGObject dependencies..."
sudo apt install -y libglib2.0-dev libcairo2-dev libgirepository1.0-dev \
    gir1.2-gtk-3.0 python3-dev ninja-build

echo "Ensuring latest Meson version is installed..."
pip install --upgrade meson

echo "Installing PyGObject via pip..."
pip install pycairo --no-cache-dir
pip install pygobject --no-cache-dir

echo "Installation of all dependencies completed successfully."
