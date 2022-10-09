package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {
    public Control(Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer) {
        super(telemetry, hardwareMap, timer);
    }

    public void Extend4Bar() {
        //extend code here
    }

    public enum BarPosition {
        HIGH(0), //TODO: calibrate constants
        MIDDLE(1),
        LOW(2);

        public final int position;

        BarPosition(int position) {
            this.position = position;
        }
    }

}