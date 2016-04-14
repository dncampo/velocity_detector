#!/bin/bash

# Primero se compila la clase
g++ -c -ggdb Blob.cpp
g++ -c -ggdb -std=c++11 -g main.cpp

#luego se compila el main, y se lo enlaza al objeto anterior
g++ -o main main.o Blob.o `pkg-config --libs --cflags opencv` -ldl

