package render;

import render.*;

public class Mesh {

    private int vao;
    private int vertices;
    private int texture = 0;

    public Mesh(int vao, int vertex) {
        this.vao = vao;
        this.vertices = vertex;
    }

    public int getVaoID() {
        return vao;
    }

    public int getVertexCount() {
        return vertices;
    }

    public Mesh addTexture(String texture) {
        this.texture = Texture.loadTexture(texture);

        return this;
    }

    public int getTexture() {
        return this.texture;
    }
}
