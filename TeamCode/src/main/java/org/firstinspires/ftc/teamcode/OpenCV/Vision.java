package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag.AprilTagPipeline;

public class Vision extends BaseDetector {

    private AprilTagPipeline aprilTagPipeline;

    public enum Pipeline {AprilTag}

    public Vision(LinearOpMode op, Pipeline pipeline) {
        super(op);

        aprilTagPipeline = new AprilTagPipeline(aprilTagPipeline.getFx(), aprilTagPipeline.getFy(), aprilTagPipeline.getCx(),aprilTagPipeline.getCy(),aprilTagPipeline.getTagSize());

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline ==Pipeline.AprilTag) {
            setPipeline(aprilTagPipeline);
        }
    }

    public AprilTagPipeline getAprilTagPipe() {
        return aprilTagPipeline;
    }
}
