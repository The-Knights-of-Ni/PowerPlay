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
        HIGH(5230, 1.0), //TODO: calibrate constants
        MIDDLE(3710, 1.0),
        LOW(2475, 1.0),
        PICKUP(0, 1.0);

        public final int position;
        public final double power;

        BarState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

    public enum ClawState {
        CLOSED(0.72),
        OPEN(0.63);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    public enum ArmState {
        DROPOFF(0.19),
        PICKUP(0);

        public final double position;

        ArmState(double position) {
            this.position = position;
        }
    }

    public enum ClawAngleState {
        PICKUP(0),
        DROPOFF(0.675);

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

    public void initDevices() {
        this.extendBar(BarState.PICKUP);
        this.toggleArm(ArmState.PICKUP);
        this.toggleClawAngle(ClawAngleState.PICKUP);
        this.toggleClaw(ClawState.CLOSED);
    }

    public void extendBar(BarState position) {
        this.bar.setPower(position.power);
        this.bar.setTargetPosition(position.position);
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

    public void deploy(BarState barState) {
        this.extendBar(barState);
        this.toggleClawAngle(ClawAngleState.DROPOFF);
        this.toggleArm(ArmState.DROPOFF);
    }
    public void retract() {
        this.toggleClawAngle(ClawAngleState.PICKUP);
        this.toggleArm(ArmState.PICKUP);
        this.extendBar(BarState.PICKUP);
    }
}
