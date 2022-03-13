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

    public AprilTagDetector(LinearOpMode op) {
        super(op, Vision.Pipeline.AprilTag);

        pipeline = new AprilTagPipeline();
        setPipeline(pipeline);
    }

    public void runAprilTag() {
        pipeline.runAprilTag();
    }

    public double[] getLocation() {
        return pipeline.getLocation();
    }

    public double[] localizeRobot(double[] starting_marker) {
        return pipeline.localizeRobot(starting_marker);
    }

    public double[] getCenterOfMarker() {
        return pipeline.getCenterOfMarker();
    }

    public double[] getCenterOfMarker(double tagX, double tagY, double tagTheta, double tagID) {
        return pipeline.getCenterOfMarker(tagX, tagY, tagTheta, tagID);
    }
}
