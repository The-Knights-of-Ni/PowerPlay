package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Superclass to all subsystems, it does some bootstrapping for them (Vision, Control, and Drive)
 */
public abstract class Subsystem {
    // protected instead of private because of inheritance
    protected final Telemetry telemetry;
    protected final ElapsedTime timer;
    protected final HardwareMap hardwareMap;

    /**
     * inits with telemetry, the hardware map, and the timer.
     * @param telemetry The telemetry for logging
     * @param hardwareMap The hardware map
     * @param timer The elapsed timer
     */
    public Subsystem(Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.timer = timer;
    }
}
