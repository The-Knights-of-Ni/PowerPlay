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

    private DcMotorEx extendBar;
    private Servo flipServo;
    private Servo clawServo;

    public enum BarState {
        HIGH(0, 0.5), //TODO: calibrate constants
        MIDDLE(1, 0.5),
        LOW(2, 0.5),
        Pickup(0, 0.5);

        public final int position;
        public final double power;

        BarState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

    public Control(Telemetry telemetry, Servo flipServo, Servo clawServo) {
        super(telemetry, "control");
        this.flipServo = flipServo;
        this.clawServo = clawServo;
    }

    public Control(Telemetry telemetry) { // TODO: Delete this
        super(telemetry, "control");
    }

    public void extend4Bar(BarState position) {
        this.turn4Bar(position);
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

    private void turn4Bar(BarState position) {
        this.extendBar.setTargetPosition(position.position);
        this.extendBar.setPower(position.power);
        this.extendBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void turnFlipServo(double position) {
        this.flipServo.setPosition(position);
    }

    private void turnClawServo(double position) {
        this.clawServo.setPosition(position);
    }

    private void turnFlipServoThread(double position) {
        FlipServoMoveThread p = new FlipServoMoveThread(flipServo, position);
        new Thread(p).start();
    }

    private void turnClawServoThread(double position) {
        ClawServoMoveThread p = new ClawServoMoveThread(clawServo, position);
        new Thread(p).start();
    }

    public void turn4BarWithClaw(BarState position) {
        turnFlipServoThread(position.position);
        if (position.position > 0.25) {
            turnClawServoThread(1.25 - position.position);
        }
        else {
            turnClawServoThread(0.25 - position.position);
        }
    }
}
