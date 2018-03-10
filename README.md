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
 - LibrePilot
   - TODO: Write a script to automate this
   - Pull latest librepilot
   - `git checkout next`
   - `cd python`
   - `sed -i '' 's/"".join(map(chr,header))/("".join(map(chr,header))).encode()/g' librepilot/uavtalk/uavtalk.py`
   - `sed -i '' 's/"".join(map(chr,data))/("".join(map(chr,data))).encode()/g' librepilot/uavtalk/uavtalk.py`
   - `sed -i '' 's/serial.write(chr(crc.read()))/serial.write(chr(crc.read()).encode())/g' librepilot/uavtalk/uavtalk.py`
   - `2to3 -wnv .`
   - `python setup.py build`
   - `python setup.py install`
 - PySerial
   - `pip install pyserial`
   
To install all dependencies available on pip, run this command:

`pip install opencv-python numpy Pillow PIMS matplotlib moviepy`