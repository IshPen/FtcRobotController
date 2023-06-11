package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive {
    private XyhVectorSA targetPosition = new XyhVectorSA(200, 200, Math.toRadians(0), "S");
    //private XyhVector pos;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;

    public MecanumDrive (DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightRear, DcMotorEx rightFront){
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.rightFront = rightFront;
        //this.pos = pos;
    }
    public void DriveForAccuracy(XyhVectorSA currentPath){

    }

    public boolean targetPositionReached(XyhVector pos, XyhVectorSA targetPosition) {
        double x1 = pos.getX();
        double x2 = targetPosition.getX();
        double y1 = pos.getY();
        double y2 = targetPosition.getY();
        double linDistBetweenTargetPosAndCurrPos = (Math.pow((y2-y1), 2) + Math.pow((x2-x1), 2));
        if (linDistBetweenTargetPosAndCurrPos < 5){ // If the linear dist between the target position and current position is less than 5 in
            return true;
        }
        else {
            return false;
        }
    }

    public void DriveForSpeed(XyhVector pos, XyhVectorSA target) { // Pass the current position of the robot, and the target destination
        double targetX = target.getX();
        double targetY = target.getY();
        double targetT = target.getTheta();
        double cPosX = pos.getX();
        double cPosY = pos.getY();
        double cPosT = pos.getTheta();

        double distToX = targetX - cPosX;
        double distToY = targetY - cPosY;
        double pivotCorrection = targetT - cPosT; //Also distToT

        double robotMovementAngle = Math.toDegrees(Math.atan2(distToX, distToY));

        double robot_movement_x_component = calcX(robotMovementAngle, 1);
        double robot_movement_y_component = calcY(robotMovementAngle, 1);
        //TODO: Set Motor Values to X and Y Components, and Pivot Correction
    }
    public void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void HaltDrive() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
    private double calcX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    private double calcY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }}
