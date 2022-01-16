package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.AntiTip;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

//Config in Constants.java
@TeleOp(name = "Anti Test")
public class AntiTipTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AntiTip antiTip = new AntiTip(this);

        while (opModeIsActive()){
            antiTip.update();

            addPacket("axis 1", antiTip.currentOrientation[0]);
            addPacket("axis 2", antiTip.currentOrientation[1]);
            addPacket("tipping", antiTip.tipping);
            sendPacket();
        }
    }
}
