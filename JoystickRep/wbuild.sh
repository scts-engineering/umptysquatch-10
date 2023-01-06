#!/bin/sh

mkdir -p build
javac -cp "lib/lwjgl-opengl.jar"\;"lib/lwjgl-glfw.jar"\;"lib/lwjgl.jar"\;"lib/lwjgl-stb.jar"\;"lib/joml-1.10.5.jar" $(find | grep 'java$') -d build
cp -r assets build
mkdir -p build/META-INF
echo "Main-Class: Main" > build/META-INF/MANIFEST.MF
cd build
zip -qr ../game.jar .
cd ..
