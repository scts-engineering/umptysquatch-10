package util;

import org.joml.Vector3f;
import java.util.Scanner;

public class PositionGetter {

    private Scanner s;
    private Vector3f position;
    
    public PositionGetter() {
        this.s = new Scanner(System.in);
        position = new Vector3f();
        
    }
    
    public Vector3f getPosition() {
        return this.position;
    }
    
    public void tick() {
        if(s.hasNextLine()) {
            String str = s.nextLine();
            String[] coordsStrs = str.split(" ");
            System.out.println("Got " + coordsStrs[0] + " and " + coordsStrs[1]);
            float x = Float.parseFloat(coordsStrs[0]);
            float y = Float.parseFloat(coordsStrs[1]);
            System.out.println("X: " + x +  ", Y: " + y);
            this.position = new Vector3f(x, y, 0);
        }
    }
}
