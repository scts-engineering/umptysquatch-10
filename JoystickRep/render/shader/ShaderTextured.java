package render.shader;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class ShaderTextured extends Shader {

    private int locationProjection;

    private int locationTransformation;

    public ShaderTextured() {
        super("Textured.vs", "Textured.fs");
    }

    @Override
    protected void bindAttributes() {

        super.bindAttribute(0, "position");
        super.bindAttribute(1, "uvs");

    }

    public void setProjection(Matrix4f mat) {

        this.loadMatrix(locationProjection, mat);

    }

    public void setTransformation(Matrix4f mat) {

        this.loadMatrix(locationTransformation, mat);

    }

    @Override
    protected void getAllUniformLocations() {

        this.locationProjection = this.getUniformLocation("projection");
        this.locationTransformation = this.getUniformLocation("transformation");

    }

}
