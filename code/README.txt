*****************************************************************************************************************************
********************************************* Proyecto 2 ********************************************************************
*****************************************************************************************************************************

README
This program consisted in an optimization of the LU algorithm using SIMD instructions, and then applying the LU algorithm to
calculate the value of the currents flowing through a Matrix of resistors, which represent different paths that are traversable
by a robot, and the value of the resistors represents the ease of that path, which depends on a provided black and white image,
where the black parts are blocks and the white parts movable space. 
The program intends to find a route between two provided nodes of the map using the outgoing current with the higher absolute 
value, and by using vectors that re[resent the general movement of a particle in a forcefield.


CONTACT

This project can be found at GitHub in:
https://github.com/h4koo/ANPI_Proyecto2

______________________________________________________________________________________________________________________________
_________________________________________________Run Instructions_____________________________________________________________
______________________________________________________________________________________________________________________________


Unzip the project, open a terminal and change your working directory to the unzipped folder

Create a directory build:

> mkdir build;

Go into that directory

> cd build;

You can choose to build a release version with:

> cmake ../code -DCMAKE_BUILD_TYPE=Release

or a debug version with

> cmake ../code -DCMAKE_BUILD_TYPE=Debug

And build everything with

> make

The executables will be stored at build/bin.

To execute the program go to the /build/bin directory

> cd build/bin

The project will build 3 executable files (i.e. proyecto2, benchmark and tester) you can run any of these
executables by using for example

>./proyecto2

When you run proyecto2 you will only receive the demo code of the imageopening up on a viewer.

+++++++++++++++++++++++++++++++++ Benchmarks +++++++++++++++++++++++++++++++++++++++++++++++++++++

The Matrix substraction function was optimized using SIMD instructions from the SSE2 set of Intel
intrinsics functions.
In order to run the benchmark used to test the improvement of the method you can use

>./benchmark -t Matrix/Subtract

The SIMD optimization for the LU algorithm and it's solver were not implemented.

********************************** Tests *********************************************************

Several small and simple maps were created for the purpose of testig the methods.

To run all tests you can simply run

>./tester

But you can narrow your results down if you use '-t' flag to provide the test you want to run. For example:

>./tester -t ResistorGrid/Navigate

You can get a list of available test cases by running

>./tester -list_content

__________________________________________________________________________________________________________________________
______________________________________________________Dependencies________________________________________________________
__________________________________________________________________________________________________________________________

You need openCV, CMAKE and Boost to build the program

> sudo apt-get install libboost-all-dev
> sudo apt-get -y install cmake