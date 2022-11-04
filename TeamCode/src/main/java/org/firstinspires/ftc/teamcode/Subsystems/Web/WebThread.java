package org.firstinspires.ftc.teamcode.Subsystems.Web;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;


public class WebThread extends Subsystem implements Runnable {
    Web web;

    public WebThread(Telemetry telemetry) {
        super(telemetry, "web");
        init(7070);
    }

    public WebThread(Telemetry telemetry, int port) {
        super(telemetry, "web");
        init(port);
    }

    private void init(int port) {
        web = new Web(port);
    }
    @Override
    public void run() {
        web.start();
    }
}
