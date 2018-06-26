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


##Creating CRPI applications

A sample is provided in Applications/Application_CRPI_Test.
To create a new CRPI application on windows, create a new empty C/C++ project in the CRPI_lite or CRPI_VS2015 solution with linker settings and C/C++ settings copied from the sample application, adding libraries if desired here.
To create a new CRPI application on Linux, create a new folder in /Applications/[Application_Name], copy the sample makefile or generate your own makefile linking with ulapi and CRPI as needed.
To build your application in windows, it is recommended that you build libraries individually as needed. 
For example, when building the sample program for the first time, you build ulapi, then library_serial, then library_CRPI, then the sample program.  

After the initial build, you will not need to rebuild libraries that you've already build individually unless you have made changes to them.
To run your built application on windows navigate to the Debug folder and run [Application_Name].exe . 

