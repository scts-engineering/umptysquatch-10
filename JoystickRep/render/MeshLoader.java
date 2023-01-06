package render;

import org.lwjgl.*;
import org.lwjgl.glfw.*;
import org.lwjgl.opengl.*;
import org.lwjgl.system.*;

import java.nio.*;
import java.util.*;

import static org.lwjgl.glfw.Callbacks.*;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryStack.*;
import static org.lwjgl.system.MemoryUtil.*;
import org.lwjgl.BufferUtils; //For creating the FloatBuffer
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;

import render.*;

public class MeshLoader {

    //contains the Vertex positions, indices, texture coordinates, and normals
    private static List<Integer> vbos = new ArrayList<Integer>();

    //contains the VBOs (Vertex Buffer Objects) of a mesh
    private static List<Integer> vaos = new ArrayList<Integer>();

    //Buffer - A container for primitive data types (a place to store data while it is being moved)

    private static FloatBuffer createFloatBuffer(float[] data) {

        FloatBuffer buffer = BufferUtils.createFloatBuffer(data.length);
        buffer.put(data); //putting the data into write mode
        buffer.flip(); //putting the data into read mode

        return buffer;
    }

    private static IntBuffer createIntBuffer(int[] data) {

        IntBuffer buffer = BufferUtils.createIntBuffer(data.length);
        buffer.put(data);
        buffer.flip();

        return buffer;
    }

    //Method to load the float data into the VBO
    private static void storeData(int attribute, int dimensions, float[] data) {

        //int attribute signifies what the VBO is going to be used as (Vertex Position, Texture Coordinates, or Normals)
        //int dimensions tells the system whether the coordinates these numbers represent are 2D or 3D

        int vbo = GL15.glGenBuffers(); //Creates a VBO ID (C style)
        vbos.add(vbo);
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, vbo); //Loads the current VBO to store the data
        FloatBuffer buffer = createFloatBuffer(data);
        GL15.glBufferData(GL15.GL_ARRAY_BUFFER, buffer, GL15.GL_STATIC_DRAW);

        //lets OpenGL know the attribute, type, whether the vertex is normalized, the offset between the vertex attributes, and where the first vertex attribute is located in the array
        GL20.glVertexAttribPointer(attribute, dimensions, GL11.GL_FLOAT, false, 0, 0);

        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0); //Unloads the current VBO when done
    }

    //Method to load the int array into the VBO
    private static void bindIndices(int[] data) {

        //does not use any vertex attributes becuase the indices are only used to tell what order the vertexes are to be drawn in

        int vbo = GL15.glGenBuffers();
        vbos.add(vbo);
        GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, vbo);
        IntBuffer buffer = createIntBuffer(data);
        GL15.glBufferData(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer, GL15.GL_STATIC_DRAW);
    }

    //Method to create a mesh with the data created
    public static Mesh createMesh(float[] positions, float[] UVs, int[] indices) {
        int vao = genVAO();
        storeData(0, 3, positions);
        storeData(1,2,UVs);
        bindIndices(indices);
        GL30.glBindVertexArray(0);

        return new Mesh(vao, indices.length);
    }

    //Method to create a new VAO, store it in the vaos list, and prep it for storing the vertex data and indices
    private static int genVAO() {
        int vao = GL30.glGenVertexArrays();
        vaos.add(vao); //adds vao to the list of vaos
        GL30.glBindVertexArray(vao);

        return vao;

    }
}
