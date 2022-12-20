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
    private Servo flipServo;
    private Servo clawServo;

    public enum BarState {
        HIGH(159, 0.5), //TODO: calibrate constants
        MIDDLE(65, 0.5),
        LOW(13, 0.5),
        Pickup(0, 0.5);

        public final int position;
        public final double power;

        BarState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

    public Control(Telemetry telemetry, DcMotorEx bar, Servo flipServo, Servo clawServo) {
        super(telemetry, "control");

        this.flipServo = flipServo;
        this.clawServo = clawServo;
    }

    public Control(Telemetry telemetry, DcMotorEx bar) { // TODO: Delete this
        super(telemetry, "control");
        this.bar = bar;
    }

    public void extend4Bar(BarState position) {
        this.extendBar(position);
        switch (position) {
            case LOW:
                this.turnFlipServo(0);
                break;
            case MIDDLE:
                this.turnFlipServo(1);
                break;
            case HIGH:
                this.turnFlipServo(2);
                break;
        }
    }

    public void extendBar(BarState position) {
        this.bar.setTargetPosition(position.position);
        this.bar.setPower(position.power);
        this.bar.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void turnFlipServo(double position) {
        this.flipServo.setPosition(position);
    }

    private void turnClawServo(double position) {
        this.clawServo.setPosition(position);
    }


    public void turn4BarWithClaw(BarState position) {
        flipServo.setPosition(position.position);
        if (position.position > 0.25) {
            clawServo.setPosition(1.25 - position.position);
        }
        else {
            clawServo.setPosition(0.25 - position.position);
        }
    }
}
