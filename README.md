# RTIKIDQualisys
C++ code for computing in real-time IK and ID using OpenSIm and the Qualisys motion capture system.

## Dependencies

1. [BOOST](http://www.boost.org/users/download/) (Ubuntu command: sudo apt-get install libboost-dev) Tested version: 1.64.0 on Win10.
1. [XSD V4](https://codesynthesis.com/products/xsd/download.xhtml) (Ubuntu command: sudo apt-get install xsdcxx) Tested version 4 on Win10.
1. [CMAKE](https://cmake.org/download/) (Ubuntu command: sudo apt-get install cmake) Tested version: 3.10.3 on Win10.
1. [OpenSim 3.1](https://simtk.org/frs/index.php?group_id=91) Tested on Win10.

## Test

Tested on Ubuntu 12.04 and QTM (2015).
We are currently working on a windows version with the lastest QTM.
The code is as it is and some library should be missing (so may need some work for compilation) but we are trying to do a stand alone version and we will update the code when we can.

## Env variable

Add to the PATH environment variable the path to the opensim dll, QT dll and the glew dll.


C:\Users\...\glew-2.1.0\bin\Release\x64
C:\Users\...\OpenSim_install\bin
C:\Qt\Qt5.10.0\5.10.0\msvc2013_64\bin

You will have to restart the cmd or powershell after changing the environement variable.

## Executable Test

From Qualisys, to test the connection nyou can try in the powerShell:

.\bin\Debug\RTClientExample.exe

To test the the connection with marker and force plate you can use (but you need QTM to stream named marker and GRF):

.\bin\Debug\QualisysTest.exe

To test the RTIKID type: 

.\bin\Debug\TestProgMarkerThreadV2.exe -i .\cfg\Alejandro\executionIK_scale.xml



