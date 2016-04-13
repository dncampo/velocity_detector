#!/bin/bash

# Primero se compila la clase

g++ -c Blob.cpp `pkg-config --libs --cflags opencv` -ldl

#luego se compila el main, y se lo enlaza al objeto anterior
g++ -std=c++11 -o main main.cpp Blob.o `pkg-config --libs --cflags opencv` -ldl

#ejecutar
./main
 
