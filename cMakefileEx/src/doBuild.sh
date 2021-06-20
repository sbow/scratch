#!/bin/bash

# This bash script give an example of how to include relative path 
# files that aren't inside directory of exucution. In other words,
# go back and then go forward to something specific, include, compile

gcc -I../include -o myProg doExample.c hello.c
