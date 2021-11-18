package org.firstinspires.ftc.teamcode.OpenCV.AprilTag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;

@Config
public class AprilTagDetector extends BaseDetector {
    private AprilTagPipeline pipeline;

    public AprilTagDetector(LinearOpMode op, double tagsize) {
        super(op);

        pipeline = new AprilTagPipeline(tagsize);
        setPipeline(pipeline);
    }

    public double[] getTagLocations() {
        return pipeline.getLocation();
    }
}
