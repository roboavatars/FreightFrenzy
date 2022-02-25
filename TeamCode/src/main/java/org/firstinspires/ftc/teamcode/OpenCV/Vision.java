package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocatorPipeline;

public class Vision extends BaseDetector {
    private AprilTagPipeline aprilTagPipeline;
    private BarcodePipeline barcodePipeline;
    private FreightLocatorPipeline freightLocatorPipeline;
    public static double TAG_SIZE = 0.079;

    public enum Pipeline {AprilTag, Barcode}

    public Vision(LinearOpMode op, Pipeline pipeline) {
        super(op);

        aprilTagPipeline = new AprilTagPipeline(TAG_SIZE);
        barcodePipeline = new BarcodePipeline(true);
        freightLocatorPipeline = new FreightLocatorPipeline();

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.AprilTag) {
            setPipeline(aprilTagPipeline);
        } else if (pipeline == Pipeline.Barcode) {
            setPipeline(aprilTagPipeline);
        }
    }

    public AprilTagPipeline getAprilTagPipe() {
        return aprilTagPipeline;
    }

    public BarcodePipeline getBarcodePipeline() {
        return barcodePipeline;
    }

    public FreightLocatorPipeline getFreightLocatorPipeline() {
        return freightLocatorPipeline;
    }
}
