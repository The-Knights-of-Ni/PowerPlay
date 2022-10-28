package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.Servo;

public class FlipServoMoveThread implements Runnable {
    Servo flipServo;
    double targetPosition;
    public FlipServoMoveThread(Servo flipServo, double targetPosition) {
        this.flipServo = flipServo;
        this.targetPosition = targetPosition;
    }

    @Override
    public void run() {
        flipServo.setPosition(targetPosition);
    }
}
