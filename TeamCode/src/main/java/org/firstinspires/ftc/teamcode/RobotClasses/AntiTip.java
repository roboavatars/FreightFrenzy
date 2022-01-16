package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AntiTip {
    private BNO055IMU imu;
    private double[] initialOrientation;
    public double[] currentOrientation;
    public boolean tipping = false;

    public AntiTip(LinearOpMode op) {
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());


        initialOrientation = getOrientation();
    }

    public double[] update (){
        currentOrientation = getOrientation();
        double[] adjustedControls = new double[2];
        if (Math.abs(currentOrientation[0]-initialOrientation[0]) > Constants.TIP_THRESHOLD ||
                Math.abs(currentOrientation[1]-initialOrientation[1]) > Constants.TIP_THRESHOLD){
            tipping = true;
            adjustedControls[0] = (currentOrientation[0] - initialOrientation[0]) * Constants.TIP_CONTROLS_GAIN;
            adjustedControls[1] = (currentOrientation[1] - initialOrientation[1]) * Constants.TIP_CONTROLS_GAIN;
        } else {
            tipping = false;
            adjustedControls[0] = 0;
            adjustedControls [1] = 0;
        }
        return adjustedControls;
    }

    /****change based on imu orientation****/
    private double[] getOrientation() {
        double[] orientation = new double[2];
        //tipping left/right
        orientation[0] = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
        //tipping forward/back
        orientation[1] = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        return orientation;
    }



}
