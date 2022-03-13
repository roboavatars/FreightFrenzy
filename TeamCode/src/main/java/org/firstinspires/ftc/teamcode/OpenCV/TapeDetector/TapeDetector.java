package org.firstinspires.ftc.teamcode.OpenCV.TapeDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Vision;

public class TapeDetector extends BaseDetector {
    private TapeDetectorPipeline pipeline;

    public TapeDetector(LinearOpMode op) {
        super(op, Vision.Pipeline.Tape);

        pipeline = new TapeDetectorPipeline();
        setPipeline(pipeline);
    }

    public double[] getResult() {
        return pipeline.getResult();
    }
}
