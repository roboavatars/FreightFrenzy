package org.firstinspires.ftc.teamcode.Pathing;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

public class Target {
    private double xTarget;
    private double yTarget;
    private double thetaTarget;
    private double vxTarget;
    private double vyTarget;
    private double wTarget;

    //constants
    private double b = Drivetrain.b;
    private double zeta = Drivetrain.zeta;

    public Target(Pose pose) {
        xTarget = pose.x;
        yTarget = pose.y;
        thetaTarget = pose.theta;
        vxTarget = pose.vx;
        vyTarget = pose.vy;
        wTarget = pose.w;
    }

    public Target(double xTarget, double yTarget, double thetaTarget) {
        this(new Pose(xTarget, yTarget, thetaTarget, 0, 0, 0));
    }

    public Target theta(double thetaTarget) {
        this.thetaTarget = thetaTarget;
        return this;
    }

    public Target thetaW0(double thetaTarget) {
        this.thetaTarget = thetaTarget;
        this.wTarget = 0;
        return this;
    }

    public Target vx(double vxTarget) {
        this.vxTarget = vxTarget;
        return this;
    }

    public Target vy(double vyTarget) {
        this.vyTarget = vyTarget;
        return this;
    }

    public Target w(double wTarget) {
        this.wTarget = wTarget;
        return this;
    }


    public Target b(double b) {
        this.b = b;
        return this;
    }

    public Target zeta(double zeta) {
        this.zeta = zeta;
        return this;
    }

    public Pose getPose() {
        return new Pose(xTarget, yTarget, thetaTarget, vxTarget, vyTarget, wTarget);
    }

    public double b() {
        return b;
    }

    public double zeta() {
        return zeta;
    }
}
