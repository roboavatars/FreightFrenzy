package org.firstinspires.ftc.teamcode.OpenCV.FreightLocator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.Freight;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawFreight;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Freight Locator Pipeline Test")
@Disabled
public class FreightLocatorTest extends LinearOpMode {

    private FreightLocator detector;
    private Drivetrain dt;
    private ArrayList<Freight> freights;

    @Override
    public void runOpMode() {
        dt = new Drivetrain(this, 111, 63, PI/2);
        detector = new FreightLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            freights = detector.getFreights(dt.x, dt.y, dt.theta);

            for (int i = 0; i < freights.size(); i++) {
                if (i == 0) {
                    drawFreight(freights.get(i), "green");
                } else if (i == 1) {
                    drawFreight(freights.get(i), "yellow");
                } else if (i == 2) {
                    drawFreight(freights.get(i), "red");
                }
            }
            drawDrivetrain(dt.x, dt.y, dt.theta, "black");
            drawField();

            addPacket("Freights", freights);
            sendPacket();
        }

        detector.stop();
    }
}
