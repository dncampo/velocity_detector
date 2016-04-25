#!/bin/bash
OPT=-Ofast
# Primero se compila la clase
g++ -Wall $OPT -c Blob.cpp
g++ -Wall $OPT -c -std=c++11 main.cpp

#luego se compila el main, y se lo enlaza al objeto anterior
g++ $OPT -o main main.o Blob.o -lopencv_highgui -lopencv_imgproc -lopencv_core

