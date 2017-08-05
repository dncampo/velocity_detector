#!/bin/bash

compile_flags=""
arch="$(uname -m)"
echo "$arch";
if [[ $arch == arm* ]] ; then
	echo "Compiling in Raspberry/ARM"
	compile_flags='-lopencv_highgui -lopencv_imgproc -lopencv_core'
elif [[ "$arch" == x86_64* ]] ; then
	echo "Compiling in GNU/Linux @X86_64"
	compile_flags=`pkg-config --libs --cflags opencv`
else
	echo "Unknown architecture. Exiting"
	exit 1
fi

OPT=-Ofast
# Primero se compila la clase
g++ -Wall $OPT -c Blob.cpp
g++ -Wall $OPT -c -std=c++11 main.cpp

#luego se compila el main, y se lo enlaza al objeto anterior
g++ $OPT -o main main.o Blob.o $compile_flags

