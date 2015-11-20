3D Object Reconstruction from<br>Hand-Object Interactions
=====

General Info
----
The source code files in this directory are an implementation of the pipeline presented in the paper:

**3D Object Reconstruction from Hand-Object Interactions**<br>
**Tzionas D., Gall J.**<br>
**International Conference on Computer Vision 2015 (ICCV'15)**<br>

**http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning**

The code was tested using Ubuntu 12.04, PCL 1.7, OpenCV 2.4.8.


Dependencies
----

CMake, OpenCV, Eigen3, libconfig++8-dev, QT4, PCL (also installes Boost):

    sudo apt-get update
    sudo apt-get install build-essential
    sudo apt-get install libopencv-dev
    sudo apt-get install libeigen3-dev
    sudo apt-get install libconfig++8-dev
    sudo apt-get install qt4-dev-tools libqt4-dev libqt4-core libqt4-gui

    sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
    sudo apt-get update
    sudo apt-get install libpcl-all

CPU_TSDF								<br>
The source code is included in the directory				<br>
    ./3rd_party
Please refer to this location for the README.md and LICENSE files. 	<br>
Compilation should be performed at this point as described in the README.md, followed by a:
    sudo make install
Original source: https://github.com/sdmiller/cpu_tsdf

==============================================================

Build
----

   mkdir build
   cd build
   cmake ..
   make

==============================================================

Configure
----

Please download from the project's website			<br>
http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning/		<br>
- the *Frames*               (e.g. http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning/Dataset/Frames___All_Files___No_Detections/001.zip) and
- the *Models/Motion/Camera* (e.g. http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning/Dataset/Models/SeqViewing/001.zip)
file for each sequence (001 or 002 or 003 or 004).		<br>
Please:
- unzip them in a different location (to avoid name conflicts)
- do NOT change the name of the exported folder (should be 001 or 002 or 003 or 004)
- enter the names of directories where the zips were unzipped in the file 
./config/CONFIG_PATHs.txt

==============================================================

Run
----

The paper includes the reconstruction of 4 objects (Water Bottle, Bowling Pin, Small Bottle, Sphere).

To run the reconstruction for an object run one of the following scripts from the current directory:

    sh ./SCRIPT__1__WaterBottle.sh
    sh ./SCRIPT__2__BowlingPin.sh
    sh ./SCRIPT__3__SmallBottle.sh
    sh ./SCRIPT__4__Sphere.sh


