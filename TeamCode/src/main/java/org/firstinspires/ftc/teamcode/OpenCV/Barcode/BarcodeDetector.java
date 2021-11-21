package org.firstinspires.ftc.teamcode.OpenCV.Barcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;

public class BarcodeDetector extends BaseDetector {
    private BarcodePipeline pipeline;

    public BarcodeDetector(LinearOpMode op, boolean isRed) {
        super(op);

        pipeline = new BarcodePipeline(isRed);
        setPipeline(pipeline);
    }

    public BarcodePipeline.Case getResult() {
        return pipeline.getResult();
    }
}
