package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Util.Vector;


public class VisionCorrection {
    Thread thread;
    public VisionCorrection(HardwareMap hardwareMap) {
        WebcamName cameraName = hardwareMap.get(WebcamName.class, Vision.WEBCAM_NAME);
        thread = new Thread(new ModelThread(cameraName));
    }

    public void correct() {
        thread.start();
        VisionThreadData.getVTD().setClose(true);
        while (!thread.isInterrupted()) {}
            VisionThreadData.getVTD().setClose(false);
    }

    private Vector calcDistanceFromBoundingBox(BoundingBox currentPosition, BoundingBox theoreticalPosition) {
        return currentPosition.transformTo(theoreticalPosition);
    }
}
