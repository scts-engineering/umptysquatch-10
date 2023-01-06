package opengl;

import render.*;
import render.shader.*;
import util.FPSCounter;
import util.PositionGetter;

public class Main {

	public static Window window;

	public void run() {

		window = new Window(400, 400);

		loop();

        window.terminate();

	}

	private void loop() {

		//set the vertices and indices of the desired shape to be drawn to the screen
		float[] vertices1 = {-100f,-100f,0f,
                100f,-100f,0f,
                100f,100f,0f,
                -100f,100f,0f};

        int[] indices = {0,1,2,0,3,2};

        float[] uvs = {0.25f,0.25f,
                0.75f,0.25f,
                0.75f,0.75f,
                0.25f,0.75f};

        Mesh mesh = MeshLoader.createMesh(vertices1, uvs, indices).addTexture("assets/shrek.png");

        Render render = new Render();
        
        PositionGetter pGetter = new PositionGetter();

		while (!window.shouldClose()) {

            FPSCounter.startCounter();

            render.cleanup();
            
            pGetter.tick();

            render.render(mesh, pGetter.getPosition());

			window.update();

			FPSCounter.stopAndPost();

		}
	}

	public static void main(String[] args) {

		new Main().run();

	}

}
