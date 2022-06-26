package org.firstinspires.ftc.teamcode.OpenCV.Barcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.opencv.core.Mat;

public class BarcodeDetector extends BaseDetector {
    private BarcodePipeline pipeline;

    public BarcodeDetector(LinearOpMode op, boolean isRed, boolean isWarehouse) {
        super(op, Vision.Pipeline.Barcode);

        pipeline = new BarcodePipeline(isRed, isWarehouse);
        setPipeline(pipeline);
    }

    public BarcodePipeline.Case getResult() {
        return pipeline.getResult();
    }

    public Mat processFrame(Mat input) {
        return pipeline.processFrame(input);
    }
}
