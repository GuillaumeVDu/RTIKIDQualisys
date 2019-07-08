# RTIKIDQUalisys
C++ code for computing in real-time IK and ID using OpenSIm and the Qualisys motion capture system.

## Dependencies

1. [BOOST](http://www.boost.org/users/download/) (Ubuntu command: sudo apt-get install libboost-dev) Tested version: 1.64.0 on Win10 and Win7./!\ 1.66 does not work.
1. [XSD V4](https://codesynthesis.com/products/xsd/download.xhtml) (Ubuntu command: sudo apt-get install xsdcxx)
1. [CMAKE](https://cmake.org/download/) (Ubuntu command: sudo apt-get install cmake) Tested version: 3.8.2 on Win10 and 3.10 on Win7. 3.14 and 3.12 does not work with OpenSim 3.1 (due to "$ENV{ProgramFiles(x86)}"). 
1. [OpenSim 3.3](https://simtk.org/frs/index.php?group_id=91) Not mandatory (for creating the spline coefficients). Tested on Win10.

## Test

Tested on Ubuntu 12.04 and QTM (2015).
We are currently working on a windows version with the lastest QTM.
The code is as it is and some library should be missing (so may need some work for compilation) but we are trying to do a stand alone version and we will update the code when we can.
