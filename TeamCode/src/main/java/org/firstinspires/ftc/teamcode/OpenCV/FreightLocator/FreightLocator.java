package org.firstinspires.ftc.teamcode.OpenCV.FreightLocator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Freight;

import java.util.ArrayList;

@Config
public class FreightLocator extends BaseDetector {
    private FreightLocatorPipeline pipeline;

    public static double minX = 0;
    public static double minY = 72;
    public static double maxX = 144;
    public static double maxY = 144;

    public FreightLocator(LinearOpMode op) {
        super(op);

        pipeline = new FreightLocatorPipeline();
        setPipeline(pipeline);
    }

    // Return freights
    public ArrayList<Freight> getRawFreights() {
        return new ArrayList<>(pipeline.getRawFreights());
    }

    // Return a list of coordinate-filtered freight
    public ArrayList<Freight> getFreights(double robotX, double robotY, double robotTheta) {
        return pipeline.getFilteredFreight(robotX, robotY, robotTheta);
    }
}
