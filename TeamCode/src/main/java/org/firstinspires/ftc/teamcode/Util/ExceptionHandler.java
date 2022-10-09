package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Thread.UncaughtExceptionHandler;

public class ExceptionHandler implements UncaughtExceptionHandler {

    Telemetry telemetry;
    @Override
    public void uncaughtException(Thread t, Throwable e) {
        telemetry.addData("Thread Death Message ","Caught Exception on Thread " + t.getId() + ", " + "Info: " + e.getClass().getName() + " " + e.getMessage() + ", StackTrace: " + e.getStackTrace() + ", Thread shutting down");
        t.interrupt();
        if (t.isAlive()) {
            t.stop();
        }
    }
}
