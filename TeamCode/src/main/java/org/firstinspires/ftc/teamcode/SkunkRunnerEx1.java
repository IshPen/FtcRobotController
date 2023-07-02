package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCEL;

import java.util.ArrayList;
import java.util.List;

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

        MecanumDrive drive = new MecanumDrive(leftFront, leftRear, rightRear, rightFront);

        leftEncoder = leftFront;
        rightEncoder = leftRear;
        frontEncoder = rightRear;
        backEncoder = rightFront;

        //stopMotors();
        drive.HaltDrive();
        drive.resetDriveEncoders();

        int oldRightPosition = 0;
        int oldLeftPosition = 0;
        int oldFrontPosition = 0;
        int oldBackPosition = 0;

        int currentRightPosition = rightEncoder.getCurrentPosition();
        int currentLeftPosition = leftEncoder.getCurrentPosition();
        int currentFrontPosition = frontEncoder.getCurrentPosition();
        int currentBackPosition = backEncoder.getCurrentPosition();

        XyhVector START_POS = new XyhVector(0, 0, Math.toRadians(0));
        XyhVector pos = new XyhVector(START_POS.getX(), START_POS.getY(), START_POS.getTheta());

        Odometry tracker = new Odometry(leftEncoder, rightEncoder, frontEncoder, backEncoder, pos, currentLeftPosition, currentRightPosition, currentFrontPosition, currentBackPosition);

        List<XyhVectorSA> mutableList = new ArrayList();
        mutableList.add(new XyhVectorSA(72, 0, Math.toRadians(0), "S"));
        mutableList.add(new XyhVectorSA(72, 72, Math.toRadians(0), "S"));

        for (int i=0; i<mutableList.size();i++){
            XyhVectorSA currentPath = mutableList.get(i);

            // Get the starting point at which you are starting a path
            tracker.updateOdometry(currentLeftPosition, currentRightPosition, currentFrontPosition, currentBackPosition);
            XyhVector startPosition = tracker.getPos();
            ElapsedTime startTime = new ElapsedTime();

            while (drive.targetPositionReached(pos, currentPath) == false) {
                currentLeftPosition = tracker.getcLP();
                currentRightPosition = tracker.getcRP();
                currentFrontPosition = tracker.getcFP();
                currentBackPosition = tracker.getcBP();
                tracker.updateOdometry(currentLeftPosition, currentRightPosition, currentFrontPosition, currentBackPosition);

                pos = tracker.getPos(); // This is the position of the robot currently in inches

                if (currentPath.getType() == "S"){
                    drive.DriveForSpeed(pos, currentPath);
                }
                else if(currentPath.getType() == "A"){
                    drive.DriveForAccuracy(pos, startPosition, currentPath, startTime, MAX_VEL, MAX_ACCEL);
                }

                telemetry.addData("X Position", pos.getX());
                telemetry.addData("Y Position", pos.getY());
                telemetry.addData("Theta", pos.getTheta());
            }
            drive.HaltDrive(); // Brake motors when reaching final destination
        }
    }
}
