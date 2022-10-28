package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServoMoveThread implements Runnable {
    Servo clawServo;
    double targetPosition;

    public ClawServoMoveThread(Servo clawServo, double targetPosition) {
        this.clawServo = clawServo;
        this.targetPosition = targetPosition;
    }

    @Override
    public void run() {
        clawServo.setPosition(targetPosition);
    }
}
