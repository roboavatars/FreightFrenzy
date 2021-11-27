package org.firstinspires.ftc.teamcode.RobotClasses.whitetapedetectionstuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorForTapeDetection {
    private ColorSensor colorSensor;
    public double THRESHOLD = 20;
    private boolean hitTape;
    public boolean isNull;

    double x,y,theta;

    public ColorSensorForTapeDetection(LinearOpMode op){
        colorSensor = op.hardwareMap.colorSensor.get("color");
        isNull = true;
        hitTape = false;
    }

    public void update(double x, double y, double theta){
        if (white() && !hitTape){
            this.x = x;
            this.y = y;
            this.theta = theta;
            isNull = false;
            hitTape = true;
        } else if (!white()){
            hitTape = false;
        }
    }

    private boolean white(){
        return colorSensor.alpha()>THRESHOLD;
    }
}
