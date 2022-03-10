package org.firstinspires.ftc.teamcode.OpenCV.AprilTag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Vision;

public class AprilTagDetector extends BaseDetector {
    private AprilTagPipeline pipeline;

    public AprilTagDetector(LinearOpMode op, double tagsize) {
        super(op, Vision.Pipeline.AprilTag);

        pipeline = new AprilTagPipeline(tagsize);
        setPipeline(pipeline);
    }

    public double[] getTagLocations() {
        return pipeline.getLocation();
    }
}
