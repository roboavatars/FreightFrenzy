package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class BaseDetector {
    private OpenCvCamera cam;
    private Vision.Pipeline pipeline;

    public BaseDetector(LinearOpMode op, Vision.Pipeline pipeline) {
        this.pipeline = pipeline;
        String cameraName = "";
        if (pipeline == Vision.Pipeline.Freight || pipeline == Vision.Pipeline.Tape)
            cameraName = "Freight Webcam";
        else cameraName = "TSE Webcam";

        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);
    }

    public void start() {
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, pipeline == Vision.Pipeline.Freight ? OpenCvCameraRotation.SIDEWAYS_RIGHT : OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                Robot.log("Open Camera Error Code " + errorCode);
            }
        });
        FtcDashboard.getInstance().startCameraStream(cam, 0);
    }

    public void stop() {
        cam.stopStreaming();
        cam.closeCameraDevice();
        FtcDashboard.getInstance().stopCameraStream();
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        cam.setPipeline(pipeline);
    }

    public int getFrameCount() {
        return cam.getFrameCount();
    }

    public double getFPS() {
        return cam.getFps();
    }
}
