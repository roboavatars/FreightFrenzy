package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.JankAprilTag.BarcodePipeline;

public class Vision extends BaseDetector {
    private AprilTagPipeline aprilTagPipeline;
    private BarcodePipeline jankAprilTagPipeline;
    public static double TAG_SIZE = 0.079;

    public enum Pipeline {AprilTag, JankAprilTag}

    public Vision(LinearOpMode op, Pipeline pipeline) {
        super(op);

        aprilTagPipeline = new AprilTagPipeline(TAG_SIZE);
        jankAprilTagPipeline = new BarcodePipeline(true);

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.AprilTag) {
            setPipeline(aprilTagPipeline);
        } else if (pipeline == Pipeline.JankAprilTag) {
            setPipeline(aprilTagPipeline);
        }
    }

    public AprilTagPipeline getAprilTagPipe() {
        return aprilTagPipeline;
    }
    public BarcodePipeline getJankAprilTagPipe() {
        return jankAprilTagPipeline;
    }
}
