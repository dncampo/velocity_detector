#!/bin/bash

# Primero se compila la clase
g++ -Wall -c Blob.cpp
g++ -Wall -c -std=c++11 main.cpp

#luego se compila el main, y se lo enlaza al objeto anterior
g++ -o main main.o Blob.o `pkg-config --libs --cflags opencv`

