package org.firstinspires.ftc.teamcode.OpenCV.JankAprilTag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;

public class JankAprilTagDetector extends BaseDetector {
    private JankAprilTagPipeline pipeline;

    public JankAprilTagDetector(LinearOpMode op) {
        super(op);

        pipeline = new JankAprilTagPipeline();
        setPipeline(pipeline);
    }

    public JankAprilTagPipeline.Case getResult() {
        return pipeline.getResult();
    }
}
