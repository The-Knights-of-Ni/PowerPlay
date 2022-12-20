package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;


/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {

    private DcMotorEx bar;
    private Servo claw;
    private Servo clawAngle;
    private Servo arm;

    public enum BarState {
        HIGH(159, 1.0), //TODO: calibrate constants
        MIDDLE(135, 1.0),
        LOW(91, 1.0),
        Pickup(0, 1.0);

        public final int position;
        public final double power;

        BarState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

    public enum ClawState {
        CLOSED(0.72),
        OPEN(0.66);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    public enum ArmState {
        CLOSED(0),
        OPEN(0.7);

        public final double position;

        ArmState(double position) {
            this.position = position;
        }
    }

    public enum ClawAngleState {
        CLOSED(0),
        OPEN(0.7);

        public final double position;

        ClawAngleState(double position) {
            this.position = position;
        }
    }

    public Control(Telemetry telemetry, DcMotorEx bar, Servo claw, Servo clawAngle, Servo arm) {
        super(telemetry, "control");
        this.bar = bar;
        this.claw = claw;
        this.clawAngle = clawAngle;
        this.arm = arm;
    }

//    public void extend4Bar(BarState position) {
//        this.extendBar(position);
//        switch (position) {
//            case LOW:
//                this.turnFlipServo(0);
//                break;
//            case MIDDLE:
//                this.turnFlipServo(1);
//                break;
//            case HIGH:
//                this.turnFlipServo(2);
//                break;
//        }
//    }

    public void extendBar(BarState position) {
        this.bar.setTargetPosition(position.position);
        this.bar.setPower(position.power);
        this.bar.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void toggleClaw(ClawState clawState) {
        this.claw.setPosition(clawState.position);
    }

    public void toggleArm(ArmState armState) {
        this.arm.setPosition(armState.position);
    }

    public void toggleClawAngle(ClawAngleState clawAngleState) {
        this.clawAngle.setPosition(clawAngleState.position);
    }

//    private void turnClawServo(double position) {
//        this.clawServo.setPosition(position);
//    }


//    public void turn4BarWithClaw(BarState position) {
//        flipServo.setPosition(position.position);
//        if (position.position > 0.25) {
//            clawServo.setPosition(1.25 - position.position);
//        }
//        else {
//            clawServo.setPosition(0.25 - position.position);
//        }
//    }
}
