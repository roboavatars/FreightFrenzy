package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.OpenCV.FreightLocator.FreightLocatorPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.TapeDetector.TapeDetectorPipeline;

public class Vision extends BaseDetector {
    private AprilTagPipeline aprilTagPipeline;
    private BarcodePipeline barcodePipeline;
    private FreightLocatorPipeline freightLocatorPipeline;
    private TapeDetectorPipeline tapeDetectorPipeline;

    public enum Pipeline {AprilTag, Barcode, Freight, Tape}

    public Vision(LinearOpMode op, Pipeline pipeline, boolean isRed, boolean isWarehouse) {
        super(op, pipeline);

        aprilTagPipeline = new AprilTagPipeline();
        barcodePipeline = new BarcodePipeline(isRed, isWarehouse);
        freightLocatorPipeline = new FreightLocatorPipeline();
        tapeDetectorPipeline = new TapeDetectorPipeline();

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.AprilTag) {
            setPipeline(aprilTagPipeline);
        } else if (pipeline == Pipeline.Barcode) {
            setPipeline(barcodePipeline);
        } else if (pipeline == Pipeline.Freight) {
            setPipeline(freightLocatorPipeline);
        } else if (pipeline == Pipeline.Tape) {
            setPipeline(tapeDetectorPipeline);
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

    public TapeDetectorPipeline getTapeDetectorPipeline() {
        return tapeDetectorPipeline;
    }
}
