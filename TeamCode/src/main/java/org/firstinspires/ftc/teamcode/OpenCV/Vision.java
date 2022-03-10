package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocatorPipeline;

public class Vision extends BaseDetector {
    private BarcodePipeline barcodePipeline;
    private FreightLocatorPipeline freightLocatorPipeline;
    private AprilTagPipeline aprilTagPipeline;
    public static double TAG_SIZE = 0.079;

    public enum Pipeline {AprilTag, Barcode, Freight}

    public Vision(LinearOpMode op, Pipeline pipeline, boolean isRed) {
        super(op, pipeline);

        barcodePipeline = new BarcodePipeline(isRed);
        freightLocatorPipeline = new FreightLocatorPipeline();
        aprilTagPipeline = new AprilTagPipeline(TAG_SIZE);

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.Barcode) {
            setPipeline(barcodePipeline);
        } else if (pipeline == Pipeline.Freight) {
            setPipeline(freightLocatorPipeline);
        } else if (pipeline == Pipeline.AprilTag) {
            setPipeline(aprilTagPipeline);
        }
    }

    public BarcodePipeline getBarcodePipeline() {
        return barcodePipeline;
    }

    public FreightLocatorPipeline getFreightLocatorPipeline() {
        return freightLocatorPipeline;
    }

    public AprilTagPipeline getAprilTagPipe() {
        return aprilTagPipeline;
    }
}
