# Team72MainCode
Code is commented and functions have funtiion header comments. If a function is not labeled it is an STM32 function.

## Functionality
This code is to be the main.c file and the place where all subsytem code and function calling will occur.
The IDE used is the STM32CubeIDE version 1.6.0. 

## Main Issues
1. The timing functions getTime and setTime are currently not returning readable output and I am unclear how to access the data in these registers.
    1. The timing function used is associtaed with the Real Time Clock of the STM32
    2. It is unclear if this is the correct timing structure to use to keep track of hours over the course of several days.

## Update as of 4/8/21
I have switched the functions over to the built in STM32 functions but I am still not sure on how to access the time. I  used a scanf statement to store the time in a specific structure but I don't know for sure how to access the hours, minutes, and seconds after that.
