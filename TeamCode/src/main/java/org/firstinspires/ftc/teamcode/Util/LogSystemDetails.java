package org.firstinspires.ftc.teamcode.Util;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class LogSystemDetails {
    public static void LogDetails(HardwareMap hardwareMap) {
        VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 5");
        String output = "Battery Voltage: " + voltSensor.getVoltage();
        Log.v("SysInfo", output);
    }
}
