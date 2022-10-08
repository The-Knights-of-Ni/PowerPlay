package org.firstinspires.ftc.teamcode.Subsystems;

public class ModelThread extends Thread {
    private Vision vision = Vision.getVision();

    @Override
    public void run() {
        vision.distance = 0;
    }
}
