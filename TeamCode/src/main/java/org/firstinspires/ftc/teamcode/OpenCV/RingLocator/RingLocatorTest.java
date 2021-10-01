package org.firstinspires.ftc.teamcode.OpenCV.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.Ring;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Ring Locator Pipeline Test")
@Disabled
public class RingLocatorTest extends LinearOpMode {

    private RingLocator detector;
    private Drivetrain dt;
    private ArrayList<Ring> rings;

    @Override
    public void runOpMode() {
        dt = new Drivetrain(this, 111, 63, PI/2);
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            dt.updatePose();

            rings = detector.getRings(dt.x, dt.y, dt.theta);

            drawDrivetrain(dt.x, dt.y, dt.theta, "black");
            drawField();

            addPacket("Rings", rings);
            sendPacket();
        }

        detector.stop();
    }
}