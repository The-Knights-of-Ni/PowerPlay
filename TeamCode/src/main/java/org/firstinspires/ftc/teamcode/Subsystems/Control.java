package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {
    private DcMotorEx extendBar;

    public Control(Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer) {
        super(telemetry, hardwareMap, timer);
        // this.extendBar = extendBar;
    }

    public void Extend4Bar(BarState position) {
        this.extendBar.setTargetPosition(position.position);
        this.extendBar.setPower(position.power);
        this.extendBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public enum BarState {
        HIGH(0, 0.5), //TODO: calibrate constants
        MIDDLE(1, 0.5),
        LOW(2, 0.5);

        public final int position;
        public final double power;

        BarState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

}