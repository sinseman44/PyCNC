![](https://cloud.githubusercontent.com/assets/8740775/26766365/14796b54-4999-11e7-8ca2-9428a45878ab.png)  

PyCNC is a free open-source high-performance G-code interpreter and
CNC/3D-printer controller. It can run on a variety of Linux-powered ARM-based
boards, such as Raspberry Pi, Odroid, Beaglebone and others. This gives you a
flexibility to pick a board you are most familiar with, and use everything
Linux has to offer, while keeping all your G-code runtime on the same board
without a need to have a separate microcontroller for real-time operation.
Our choice of Python as main programming language significantly reduces code
base compared to C/C++ projects, reduces boilerplate and microcontroller-specific
code, and makes the project accessible to a broader audience to tinker with.

# Realtime Motor Control in Linux?
Typically there is no way to control stepper motors from Linux runtime
environment due to the lack of real time GPIO control. Even kernel based
modules can not guarantee precise control of pulses for steppers.
However, we can use a separate hardware module, DMA (Direct Memory Access)
which provides high precision for GPIO outputs. This module can copy bytes which
represent GPIO states from RAM buffer directly to GPIO with some clock based
on main chip internal oscillator without using CPU's cores. Using such approach
this project generates impulses for moving stepper motors and that is very
precise way regardless CPU load and OS time jitter.  
This approach also allows to use Python language for this project. Typically,
Python is not good choice for real time application, but since project just
needs to set up DMA buffers and hardware will do the rest, Python become the
perfect choice for easy development of this project.

Video demo - [YouTube video](https://youtu.be/41wdmmztTNA)  
And the original video when PyCNC was just a prototype [YouTube video](https://youtu.be/vcedo59raS4)

# Current gcode and features support
* Commands G0, G1, G2, G3, G4, G17, G18, G19, G20, G21, G28, G53, G90, G91, G92,
M2, M3, M5, M30, M84, M104, M105, M106, M107, M109, M114, M140, M190 are
supported. Commands can be easily added, see [gmachine.py](./cnc/gmachine.py)
file.
* Four axis are supported - X, Y, Z, E.
* Circular interpolation for XY, ZX, YZ planes is supported.
* Spindle with rpm control is supported.
* Extruder and bed heaters are supported.
* Hardware watchdog.

# Watchdog
PyCNC uses one of DMA channels as hardware watchdog for safety purpose. If
board, OS or PyCNC hangs this watchdog should disable all GPIO pins(by
switching them into input state, for RPi this would be GPIO0-29) in 15 seconds.
Since there is a high current and dangerous devices like heated bed, extruder
heater, this feature should prevent uncontrollable overheating. But don't count
on such software features too much, they can hang too or output MOSFET become
shorted, use hardware protection like thermal cutoff switches in your machines.

# Hardware
Currently, this project supports Raspberry Pi 1-3. Developed and tested with
RPI1.2b. And there is a way to add new boards. See [hal.py](./cnc/hal.py) file.  
_Note: Current Raspberry Pi implementation uses the same resources as on board
3.5 mm jack(PWM module), so do not use it. HDMI audio works._

# Config
All configs are stored in [config.py](./cnc/config.py) and contain hardware
properties, limitations and pin names for hardware control.  
Initially, Raspberry Pi implementation should be connected to A4988, DRV8825 or any other
stepper motor drivers with DIR and STEP pin inputs.

For this branch (l293d), Raspberry Pi implementation should be connected to L293D stepper motor drivers.

![l293d_front](https://github.com/sinseman44/PyCNC/blob/l293d/.github/images/l293d_1.png)
![l293d_back](https://github.com/sinseman44/PyCNC/blob/l293d/.github/images/l293d_2.png)

Default config is created for Raspberry Pi 1.2b and PiPlot2D system and this wiring config:

| Circuit name     | RPi pin name |  Note           |
|------------------|--------------|-----------------|
| X cmd A          | GPIO12       | to first L293d  |
| X cmd A'         | GPIO16       | to first L293d  |
| X cmd B          | GPIO20       | to first L293d  |
| X cmd B'         | GPIO21       | to first L293d  |
| X endstop        | GPIO10       |                 |
| Y cmd A          | GPIO24       | to 2nd L293d    |
| Y cmd A'         | GPIO25       | to 2nd L293d    |
| Y cmd B          | GPIO8        | to 2nd L293d    |
| Y cmd B'         | GPIO7        | to 2nd L293d    |
| Y endstop        | GPIO9        |                 |
| servo motor cmd  | GPIO18       | to sg90 servo   |
| display data 4   | GPIO6        | to HD44780      |
| display data 5   | GPIO13       | to HD44780      |
| display data 6   | GPIO19       | to HD44780      |
| display data 7   | GPIO26       | to HD44780      |
| display clk edge | GPIO5        | to HD44780      |
| display Reg Sel  | GPIO11       | to HD44780      |
| button up        | GPIO17       |                 |
| button ok        | GPIO22       |                 |
| button down      | GPIO27       |                 |

This is the PiPlot2D system based on a Raspberry Pi 1.2b, L293d Stepper motor drivers and 
DC 5V 2 phases 4 wires stepper motor recovered from old DVD-roms.
Draw dimensions : 4cm x 5cm 

![PiPlot2D](https://github.com/sinseman44/PyCNC/blob/l293d/.github/images/PiPlot2D.png)

# Usage
Just clone this repo and run `./pycnc` from repo root. It will start in
interactive terminal mode where gcode commands can be entered manually.  
To run file with gcode commands, just run `./pycnc filename`.  
Optionally, `pycnc` can be installed. Run
```bash
sudo pip install .
```
in repo root directory to install it. After than, `pycnc` command will be added
to system path. To remove installation, just run:
```bash
sudo pip remove pycnc
```

# Performance notice
Pure Python interpreter would not provide great performance for high speed
machines. Overspeeding setting causes motors mispulses and probably lose of
trajectory. According to my tests, Raspberry Pi 2 can handle axises with 400
 pulses on mm with top velocity ~800 mm per min. There is always way out! :)
Use JIT Python implementation like PyPy. RPi2 can handle up to 18000 mm per
minute on the machine with 80 steps per millimeter motors with PyPy.  
_Note: Raspbian has outdated PyPy version in repositories(v4.0). Moreover v4.0
has issue with `mmap` module implementation. Use PyPy v5.0+, download it for
your OS from [here](https://pypy.org/download.html)._  
PyPy installation:
```bash
wget wget https://bitbucket.org/pypy/pypy/downloads/pypy2-v5.7.1-linux-armhf-raspbian.tar.bz2
sudo mkdir /opt/pypy
sudo tar xvf pypy2-v5.7.1-linux-armhf-raspbian.tar.bz2 --directory /opt/pypy/ --strip-components=1
sudo ln -s /opt/pypy/bin/pypy /usr/local/bin/pypy
```

# Project architecture
![](https://user-images.githubusercontent.com/8740775/27770129-c8c3592c-5f41-11e7-8a9c-254d5a88ed77.png)

# Dependencies
Nothing for runtime. Just pure Python code.
For uploading to PyPi there is a need in `pandoc`:
```bash
sudo dnf install pandoc
sudo pip install pypandoc
```

# GCode simulation
Just a link, mostly for myself :), to a nice web software for gcode files
emulation (very helpful for manual creating of gcode files):
[https://nraynaud.github.io/webgcode/](https://nraynaud.github.io/webgcode/)

# License
see [LICENSE](./LICENSE) file.

# Author
Nikolay Khabarov

