package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Odometry {
    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx frontEncoder;
    private DcMotorEx backEncoder;
    private XyhVector pos;
    private int oldLeftPosition;
    private int oldRightPosition;
    private int oldBackPosition;
    private int oldFrontPosition;

    final static double WHEEL_RADIUS = 1.8898; // in
    final static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    final static double LATERAL_DIST = 15.25; // in
    final static double LONGITUDINAL_DIST = 15.25; // in
    final static double TICKS_PER_REVOLUTION = 8192; // encoder ticks per revolution, rev encoder
    final static double in_per_tick = 2.0 * Math.PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentFrontPosition = 0;
    public int currentBackPosition = 0;

    public Odometry(DcMotorEx leftEncoder, DcMotorEx rightEncoder, DcMotorEx frontEncoder, DcMotorEx backEncoder, XyhVector pos, int currentLeftPosition, int currentRightPosition, int currentFrontPosition, int currentBackPosition) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.frontEncoder = frontEncoder;
        this.backEncoder = backEncoder;
        this.pos = pos;
        this.oldLeftPosition = currentLeftPosition;
        this.oldRightPosition = currentRightPosition;
        this.oldBackPosition = currentBackPosition;
        this.oldFrontPosition = currentFrontPosition;
    }
    public void updateOdometry(int currentLeftPosition, int currentRightPosition, int currentFrontPosition, int currentBackPosition) {
        //Set old position to the previous current position
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldFrontPosition = currentFrontPosition;
        oldBackPosition = currentBackPosition;

        //Get the new current position of encoders
        currentRightPosition = -rightEncoder.getCurrentPosition();
        currentLeftPosition = leftEncoder.getCurrentPosition();
        currentFrontPosition = frontEncoder.getCurrentPosition();
        currentBackPosition = backEncoder.getCurrentPosition();

        // Delta Encoder Positions
        int dn1 = currentLeftPosition  - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentFrontPosition - oldFrontPosition;
        int dn4 = currentBackPosition - oldBackPosition;

        // the robot has moved and turned a tiny bit between two measurements:
        //double dtheta = in_per_tick * ((dn2-dn1) / (LATERAL_DIST));
        double dthetaX = in_per_tick * ((dn2-dn1) / (LATERAL_DIST));
        double dthetaY = in_per_tick * ((dn4-dn3) / (LONGITUDINAL_DIST));

        double dtheta = (dthetaX+dthetaY)/2;

        double dx = in_per_tick * ((dn1+dn2) / 2.0);//Calculate DeltaX in Inches
        double dy = in_per_tick * (dn3 + ((dn2-dn1) / 2.0));//Calculate DeltaY in Inches

        //telemetrydx = dx;
        //telemetrydy = dy;
        //telemetrydh = dtheta;

        // small movement of the robot gets added to the field coordinate system:
        double theta = pos.getX() + (dtheta/2.0);

        // Actually set internal pos to calculated values
        pos.setX(pos.getX() + (dx*Math.cos(theta))-(dy*Math.sin(theta)));
        pos.setY(pos.getY() + (dx*Math.sin(theta))+(dy*Math.cos(theta)));
        pos.setTheta(pos.getTheta()+dtheta);
    }
    public int getcLP() {
        this.currentLeftPosition = leftEncoder.getCurrentPosition();
        return this.currentLeftPosition;
    }
    public int getcRP() {
        this.currentRightPosition = -rightEncoder.getCurrentPosition();
        return this.currentRightPosition;
    }
    public int getcFP() {
        this.currentFrontPosition = frontEncoder.getCurrentPosition();
        return this.currentFrontPosition;
    }
    public int getcBP() {
        this.currentBackPosition = backEncoder.getCurrentPosition();
        return this.currentBackPosition;
    }
    public XyhVector getPos(){return pos;}
}
