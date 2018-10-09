# ANPI_Proyecto2
Proyecto 2 de ANPI
Luis Fernando Murillo 
Jorge Aguero

*****************************************************************************************************************************
**********************************************  Tarea 4 **************************************************
*****************************************************************************************************************************

README

In this project we implemented matrix decomposition method to fine a current rute.
We used SIMD to optimizate operation.


Which are used to solve the equations Ax = b

CONTACT

If you have problems or comments with this program you
can contact please contact us.

This project can also be found at GitHub in:
https://github.com/h4koo/ANPI_Proyecto2

-----------------------------------------------------------------------------------------------------------------------
------------------------------------------- RUN INSTRUCTIONS ---------------------------------------------
-----------------------------------------------------------------------------------------------------------

Unzip the project, open a terminal and change your working directory to the unzipped folder

Create a directory build:

> mkdir build;

Go into that directory

> cd build;

You can choose to build a release version with:

> cmake ../ -DCMAKE_BUILD_TYPE=Release

or a debug version with

> cmake ../ -DCMAKE_BUILD_TYPE=Debug

And build everything with

> make

The executables will be stored at build/bin.

To execute the program go to the /build/bin directory

> cd bin

If you can, running all benchmarks of task you can use


If you can, running all test of task you can use

> ./tester 

