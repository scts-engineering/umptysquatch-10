package render;

import org.lwjgl.*;
import org.lwjgl.glfw.*;
import org.lwjgl.opengl.*;
import org.lwjgl.system.*;

import java.nio.*;

import static org.lwjgl.glfw.Callbacks.*;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryStack.*;
import static org.lwjgl.system.MemoryUtil.*;
import static org.lwjgl.glfw.GLFWWindowSizeCallback.*;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import render.*;
import render.shader.*;
import util.FPSCounter;
import opengl.Main;

public class Window {

    public long window;

    public int width, height, aspectRatio;

    private GLFWWindowSizeCallback windowSize;

    public Window(int width, int height) {

        init(width, height);

    }

    private static int MAX_MIN_DIM = 50000;

    private void init(int width, int height){

        this.width = width;
        this.height = height;
        this.aspectRatio = width / height;

		GLFWErrorCallback.createPrint(System.err).set();

        if(!GLFW.glfwInit()) {
            throw new IllegalStateException("Unable to initialize GLFW");
        }

        GLFW.glfwDefaultWindowHints();
        GLFW.glfwWindowHint(GLFW.GLFW_VISIBLE, GLFW.GLFW_FALSE);
        GLFW.glfwWindowHint(GLFW.GLFW_RESIZABLE, GLFW.GLFW_TRUE);

        window = GLFW.glfwCreateWindow(width, height, "Shrek", NULL, NULL);
        if(window == NULL) {
            throw new IllegalStateException("Unable to create GLFW Window");
        }

        GLFW.glfwSetKeyCallback(window, (window, key, scancode, action, mods) -> {});

        try(MemoryStack stack = stackPush()){
            IntBuffer pWidth = stack.mallocInt(1);
            IntBuffer pHeight = stack.mallocInt(1);

            GLFW.glfwGetWindowSize(window, pWidth, pHeight);

            GLFWVidMode vidmode = GLFW.glfwGetVideoMode(GLFW.glfwGetPrimaryMonitor());

            GLFW.glfwSetWindowPos(
                window,
                (vidmode.width() - pWidth.get(0)) / 2,
                (vidmode.height() - pHeight.get(0)) / 2
            );

            GLFW.glfwMakeContextCurrent(window);
            GLFW.glfwSwapInterval(0);
            GLFW.glfwShowWindow(window);
        }

        GLFW.glfwSetWindowSizeCallback(window, windowSize = new GLFWWindowSizeCallback() {

            @Override
            public void invoke(long window, int width, int height) {

                Main.window.height = height;
                Main.window.width = width;
                Main.window.aspectRatio = width / height;
                GL11.glViewport(0, 0, Main.window.width, Main.window.height);

            }
        });
        GL.createCapabilities();
    }
    public void update() {

        GLFW.glfwSwapBuffers(window);
        GLFW.glfwPollEvents();

    }

    public boolean shouldClose() {

        return GLFW.glfwWindowShouldClose(window);

    }

    public void terminate() {

        Callbacks.glfwFreeCallbacks(window);
        GLFW.glfwDestroyWindow(window);

        GLFW.glfwTerminate();
        GLFW.glfwSetErrorCallback(null).free();

    }

    //note that this only works for 2d shapes
    public Matrix4f getProjectionMatrix() {

        Matrix4f matrix = new Matrix4f().ortho2D(-MAX_MIN_DIM, MAX_MIN_DIM, -MAX_MIN_DIM, MAX_MIN_DIM);

        return matrix;

    }
}
