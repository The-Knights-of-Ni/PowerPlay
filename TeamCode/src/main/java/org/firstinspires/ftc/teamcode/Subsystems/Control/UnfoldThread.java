package org.firstinspires.ftc.teamcode.Subsystems.Control;

import org.firstinspires.ftc.teamcode.Robot;

public class UnfoldThread extends Thread{
    private Control control;
    public UnfoldThread(Control control) {
        this.control = control;
    }

    public void run() {
        try {
            control.toggleArm(Control.ArmState.UNFOLD);
            this.sleep(1500);
            control.toggleClawAngle(Control.ClawAngleState.PICKUP);
            this.sleep(500);
            control.toggleArm(Control.ArmState.PICKUP);
            control.toggleClaw(Control.ClawState.OPEN);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
