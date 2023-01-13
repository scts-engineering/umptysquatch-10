package render.shader;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.FloatBuffer;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import java.nio.file.Files;

public abstract class Shader{

    private int programID;
    private int vertexID;
    private int fragmentID;

    private FloatBuffer matrix = BufferUtils.createFloatBuffer(16);

    public Shader(String Vert, String Frag) { // the locations for the Vertex and fragment GLSL shader files
        vertexID = loadShader(Vert, GL20.GL_VERTEX_SHADER);
        fragmentID = loadShader(Frag, GL20.GL_FRAGMENT_SHADER);
        programID = GL20.glCreateProgram();
        GL20.glAttachShader(programID, vertexID);
        GL20.glAttachShader(programID, fragmentID);
        bindAttributes();
        GL20.glLinkProgram(programID);
        GL20.glValidateProgram(programID);
        getAllUniformLocations();
    }

    public void start() {
        GL20.glUseProgram(programID); //tells OpenGL to render the files with specified shader
    }

    public void stop() {
        GL20.glUseProgram(0); //tells OpenGL to render the files without any shaders
    }

    private static int loadShader(String file, int type) {
        StringBuilder shaderSource = new StringBuilder();
        try {
            BufferedReader reader = new BufferedReader(new FileReader("assets/shaders/"+file));
            String line;
            while((line = reader.readLine()) != null) {
                shaderSource.append(line).append("\n");
            }

            reader.close();

        }catch(IOException e) {
            System.err.println("Can't read file");
            e.printStackTrace();
            System.exit(-1);
        }


  /*      String shaderSource = "";
        try {
            shaderSource = String.join("\n", Files.readAllLines(file));
        } catch(IOException e) {
            System.err.println("Can't read file " + file);
            e.printStackTrace();
            System.exit(-1);
        } */

        int ID = GL20.glCreateShader(type);
        GL20.glShaderSource(ID, shaderSource);
        GL20.glCompileShader(ID);

        if(GL20.glGetShaderi(ID, GL20.GL_COMPILE_STATUS)==GL11.GL_FALSE) {
            System.out.println(GL20.glGetShaderInfoLog(ID, 512));
            System.err.println("Couldn't compile the shader");
            System.exit(-1);
        }

        return ID;
    }

    protected int getUniformLocation(String uniformName) {
        return GL20.glGetUniformLocation(programID, uniformName);
    }

    protected void bindAttribute(int attribute, String variableName) {
        GL20.glBindAttribLocation(programID, attribute, variableName);
    }

    protected void loadFloat(int location, float value) {
        GL20.glUniform1f(location, value);
    }

    protected void loadVector(int location, Vector3f vector) {
        GL20.glUniform3f(location, vector.x, vector.y, vector.z);
    }

    protected void loadBoolean(int location, boolean value) {
        float tovec = 0;
        if (value) {
            tovec = 1;
        }
        GL20. glUniform1f(location, tovec);
    }

    protected void loadMatrix(int location, Matrix4f value) {

        this.matrix = BufferUtils.createFloatBuffer(16);
        value.get(matrix);
        GL20.glUniformMatrix4fv(location, false, matrix);

    }

    protected abstract void bindAttributes();

    protected abstract void getAllUniformLocations();
}
