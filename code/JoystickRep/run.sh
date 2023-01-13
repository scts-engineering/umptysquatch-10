#!/bin/sh


vblank_mode=0 java -Djava.library.path=natives -cp "lib/lwjgl-opengl.jar":"lib/lwjgl-glfw.jar":"lib/lwjgl.jar":"lib/joml-1.10.5.jar":"lib/lwjgl-stb.jar":"game.jar" opengl/Main
