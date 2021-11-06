package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.RingLocator.RingLocatorPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline;

public class Vision extends BaseDetector {

    private StackHeightPipeline stackHeightPipeline;
    private RingLocatorPipeline ringLocatorPipeline;
    private AprilTagPipeline aprilTagPipeline;

    public enum Pipeline {StackHeight, RingLocator, AprilTag}

    public Vision(LinearOpMode op, Pipeline pipeline) {
        super(op);

        stackHeightPipeline = new StackHeightPipeline();
        ringLocatorPipeline = new RingLocatorPipeline();
        aprilTagPipeline = new AprilTagPipeline(aprilTagPipeline.getFx(), aprilTagPipeline.getFy(), aprilTagPipeline.getCx(),aprilTagPipeline.getCy(),aprilTagPipeline.getTagSize());

        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.StackHeight) {
            setPipeline(stackHeightPipeline);
        } else if (pipeline == Pipeline.RingLocator) {
            setPipeline(ringLocatorPipeline);
        } else if (pipeline ==Pipeline.AprilTag) {
            setPipeline(aprilTagPipeline);
        }
    }

    public StackHeightPipeline getStackPipe() {
        return stackHeightPipeline;
    }

    public RingLocatorPipeline getRingPipe() {
        return ringLocatorPipeline;
    }

    public AprilTagPipeline getAprilTagPipe() {
        return aprilTagPipeline;
    }
}
