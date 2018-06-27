# CRPI(Collaborative Robot Programming Interface)

Originally based on the Canonical Robot Command Language (CRCL), the NIST ISD Collaborative Robot Programming Interface (CRPI) is intended to provide an architecture to support the metrology of collaborative robot performance by means of commanding a myriad of robot platforms simultaneously with a singular command structure. The CRPI was developed to be an instantiation of the conceptualized model of the CRCL, and is subject to expansion, as real-world applications require additional functionality. Current development of the CRPI is focused on enabling and supporting application-driven multi-robot coordination and collaboration, and human robot interaction.

## Required Software

CRPI runs on Windows and Linux machines, though some libraries are limited on the Linux OS. 

On Windows machines, CRPI requires Visual Studio 2015 or higher. 

On Linux Machines, CRPI requires you to first install [ULAPI](https://github.com/frederickproctor/ulapi)

## Building 

On linux machines, to build all libraries for CRPI, run the following command in the main directory
> python buildLinuxCRPI.py -a

To clean your build of CRPI run
>python buildLinuxCRPI.py -c

On windows machines, you can directly debug from the sample application or the UnitTest application in the CRPI_lite visual studio solution. 


## Creating CRPI applications

A sample is provided in Applications/Application_CRPI_Test.
To create a new CRPI application on windows, create a new empty C/C++ project in the CRPI_lite or CRPI_VS2015 solution with linker settings and C/C++ settings copied from the sample application, adding libraries if desired here.
### To create a new CRPI application on Linux
 -Create a new folder in /Applications/[Application_Name] 
 -Copy the sample makefile or generate your own makefile linking with ulapi and CRPI as needed.
 
### To build your application in windows,
 -Create a new empty C/C++ project in the CRPI_lite or CRPI_VS2015 solution
 -Copy linker settings and C/C++ settings from the sample application, adding additional libraries as needed.
 -Build libaries individually, incrimentally working up to your application ie.) for the sample application build ulapi first, then libary_serial, then libary_CRPI, then Application_CRPI_Test.
 -To run your built application on windows navigate to the Debug folder and run [Application_Name].exe . 
It is recommended that you build libraries individually as needed within the CRPI visual studio solution.
After the initial build, you will not need to rebuild libraries that you've already build individually unless you have made changes to them.

## Acknowledgements
Third party libaries and code for various sensor input and processing is used in applications and sensing libaries in CRPI, including but not limited to: 
- [OpenCV](https://github.com/opencv/opencv)
- [OpenGL Ultility Toolkit (glut)](https://www.opengl.org/resources/libraries/glut/)
- [OpenNI](https://github.com/OpenNI/OpenNI)
- [OpenVR](https://github.com/ValveSoftware/openvr)
- [Ulapi](https://github.com/frederickproctor/ulapi)
- [Leap SDK](https://developer.leapmotion.com/get-started/)
- [Myo SDK](https://developer.thalmic.com/downloads)
- [Optitrak SDK](http://optitrack.com/downloads/developer-tools.html)
- [Vicon SDK](https://www.vicon.com/products/software/datastream-sdk)

## Updates
### 9/22/2017:
First public release including base CRPI code
- CRPI Math Libary
- CRPI Serial Library
- CRPI Motion Prims Libarary
- CRPI Registration Kit Libarary
- CRPI Libary
- Robot onboard code
- General ML Clustering libary

### 4/9/2018
- Linux compatability added to libaries
- Sample program provided
- Simplified visual studio solution added - CRPI lite

### 6/27/18
- Updated Liscense and Readme to comply with public repository guidelines
- Resolve Mocap merge errors

## Contact
Feel free to reach out to @mlzimmer for any questions, or post an issue to the repo.
You can also email megan.zimmerman@nist.gov
