package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(group = "drive")
public class SkunkRunnerEx1 extends LinearOpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public DcMotorEx leftEncoder, rightEncoder, frontEncoder, backEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder = leftFront;
        rightEncoder = leftRear;
        frontEncoder = rightRear;
        backEncoder = rightFront;

        stopMotors();
        resetDriveEncoders();
        int oldRightPosition = 0;
        int oldLeftPosition = 0;
        int oldFrontPosition = 0;
        int oldBackPosition = 0;

        int currentRightPosition = rightEncoder.getCurrentPosition();
        int currentLeftPosition = leftEncoder.getCurrentPosition();
        int currentFrontPosition = frontEncoder.getCurrentPosition();
        int currentBackPosition = backEncoder.getCurrentPosition();


        boolean trigger = true;
        XyhVector START_POS = new XyhVector(0, 0, Math.toRadians(0));
        XyhVector pos = new XyhVector(START_POS.getX(), START_POS.getY(), START_POS.getTheta());

        Odometry tracker = new Odometry(leftEncoder, rightEncoder, frontEncoder, backEncoder, pos, currentLeftPosition, currentRightPosition, currentFrontPosition, currentBackPosition);

        while (trigger) {
            currentLeftPosition = tracker.getcLP();
            currentRightPosition = tracker.getcRP();
            currentFrontPosition = tracker.getcFP();
            currentBackPosition = tracker.getcBP();
            tracker.updateOdometry(currentLeftPosition, currentRightPosition, currentFrontPosition, currentBackPosition);
        }
    }


    private void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }





}
