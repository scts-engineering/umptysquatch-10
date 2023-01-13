package util;

import java.lang.System.*;

final public class FPSCounter {

    private static int startTime, endTime;
    private static int frameTimes = 0;
    private static short frames = 0;

    public final static void startCounter() {
        startTime = (int)System.currentTimeMillis();
    }

    public final static void stopAndPost() {

        endTime = (int)System.currentTimeMillis();
        frameTimes = frameTimes + endTime - startTime;
        frames++;

        if(frameTimes >= 1000) {

            System.out.println("FPS: " + Long.toString(frames));
            frames = 0;
            frameTimes = 0;
            startTime = endTime;
        }
    }
}
