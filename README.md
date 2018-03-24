# AtLAS-embedded

![](https://reposs.herokuapp.com/?path=ItsTimmy/AtLAS-server&color=blue)
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

Embedded software for AuTonomous Low-Altitude Service drone

### Dependencies

Note: Before running these install commands, you should create a virtual environment so that the default `python` and
`pip` commands are version 3, instead of 2.

 - OpenCV 3.X
   - Installation:
     - Mac: Install Homebrew and run `brew install opencv`
     - Windows: [Tutorial](https://docs.opencv.org/3.2.0/d3/d52/tutorial_windows_install.html)
     - Linux: [Good luck](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)
   - Also install python bindings using `pip install opencv-python`
 - Numpy
   - `pip install numpy`
 - Pillow
   - `pip install Pillow`
 - PIMS (Python Image Sequence)
   - `pip install PIMS`
 - Matplotlib
   - `pip install matplotlib`
 - MoviePy
   - `pip install moviepy`
 - PySerial
   - `pip install pyserial`
 - GPS-python3
   - Make sure to be in the root directory of this repository when running this command, because it installs gps-python3
   from a folder rather than from PyPI
   - `pip install gps-python3/`
   
To install all dependencies available on pip, run this command:

`pip install opencv-python numpy Pillow PIMS matplotlib moviepy`