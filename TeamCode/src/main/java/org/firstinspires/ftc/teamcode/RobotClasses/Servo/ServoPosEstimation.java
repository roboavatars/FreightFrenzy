package org.firstinspires.ftc.teamcode.RobotClasses.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPosEstimation {
    public Servo servo;
    public double speed;
    public double currentPos;
    public double targetPos;

    double lastTime;

    public ServoPosEstimation(LinearOpMode op, String servoName, double initPos, double servoSpeed) {
        servo = op.hardwareMap.get(Servo.class, servoName);
        setPosition(initPos);

        currentPos = initPos;
        speed = servoSpeed;

        lastTime = System.currentTimeMillis();
    }

    public void setPosition(double pos) {
        update();
        targetPos = Math.min(Math.max(pos, 0), 1);
        servo.setPosition(targetPos);
    }

    public double getPos() {
        update();
        return currentPos;
    }

    //update the current servo position to the target position
    public void update() {
        if (targetPos > currentPos) currentPos += Math.min(speed * (System.currentTimeMillis() - lastTime), targetPos);
        if (targetPos < currentPos) currentPos -= Math.max(speed * (System.currentTimeMillis() - lastTime), targetPos);
        lastTime = System.currentTimeMillis();
    }
}
