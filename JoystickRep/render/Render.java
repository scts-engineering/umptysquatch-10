package render;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import render.*;
import render.shader.ShaderTextured;
import opengl.Main;
import util.Maths;
import util.PositionGetter;

public class Render {

    ShaderTextured shader = new ShaderTextured();
    

    float angle = 0;

    

    public void cleanup() {
        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
    }

    public void render(Mesh mesh, Vector3f position) {


        shader.start();

        shader.setProjection(Main.window.getProjectionMatrix());

        shader.setTransformation(Maths.createTransformationMatrix(position, 0, 0, 0, 1));

        GL30.glBindVertexArray(mesh.getVaoID());
        GL20.glEnableVertexAttribArray(0); //allows the vertex positions to be drawn
        GL20.glEnableVertexAttribArray(1);
        GL13.glActiveTexture(GL13.GL_TEXTURE0);
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, mesh.getTexture());

        GL11.glDrawElements(GL11.GL_TRIANGLES, mesh.getVertexCount(), GL11.GL_UNSIGNED_INT,0);
        //UNSIGNED_INT tells OpenGL to type the values of our index array, and 0 tells it which vertex to start drawing from


        GL20.glDisableVertexAttribArray(0);
        GL20.glDisableVertexAttribArray(1);
        GL30.glBindVertexArray(0);
        shader.stop();



    }

}
