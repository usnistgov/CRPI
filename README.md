# CRPI(Collaborative Robot Programming Interface)

Originally based on the Canonical Robot Command Language (CRCL), the NIST ISD Collaborative Robot Programming Interface (CRPI) is intended to provide an architecture to support the metrology of collaborative robot performance by means of commanding a myriad of robot platforms simultaneously with a singular command structure. The CRPI was developed to be an instantiation of the conceptualized model of the CRCL, and is subject to expansion, as real-world applications require additional functionality. Current development of the CRPI is focused on enabling and supporting application-driven multi-robot coordination and collaboration, and human robot interaction.

## Required Software

CRPI runs on Windows and Linux machines, though some libraries are limited on the Linux OS. 

On Windows machines, CRPI requires Visual Studio 2015 or higher. 

On Linux Machines, CRPI requires you to first install`[ULAPI](https://github.com/frederickproctor/ulapi)` 

## Building 

On linux machines, to build all libraries for CRPI, run the following command in the main directory
> python buildLinuxCRPI.py -a

To clean your build of CRPI run
>python buildLinuxCRPI.py -c

On windows machines, you can directly debug from the sample application or the UnitTest application in the CRPI visual studio solution. 
