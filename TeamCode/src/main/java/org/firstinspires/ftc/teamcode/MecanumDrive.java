package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;

public class MecanumDrive {
    private XyhVectorSA targetPosition = new XyhVectorSA(200, 200, Math.toRadians(0), "S");
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    private ElapsedTime timeInPath;
    public MecanumDrive (DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightRear, DcMotorEx rightFront){
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.rightFront = rightFront;

        //this.pos = pos;
    }

    public boolean targetPositionReached(XyhVector pos, XyhVectorSA targetPosition) {
        double x1 = pos.getX();
        double x2 = targetPosition.getX();
        double y1 = pos.getY();
        double y2 = targetPosition.getY();
        double linDistBetweenTargetPosAndCurrPos = (Math.pow((y2-y1), 2) + Math.pow((x2-x1), 2));
        if (linDistBetweenTargetPosAndCurrPos < 2){ // If the linear dist between the target position and current position is less than 2 in
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
    /*
    public void DriveForAccuracy(XyhVector pos, ElapsedTime pathTime, XyhVectorSA target) {

        // Current Positions
        double cPosX = pos.getX();
        double cPosY = pos.getY();
        double cPosT = pos.getTheta();

        // Target Positions
        double targetX = target.getX();
        double targetY = target.getY();
        double targetT = target.getTheta();

        // Distance to X and Y
        double delta_x = targetX - cPosX;
        double delta_y = targetY - cPosY;


        double pivotCorrection = targetT - cPosT; //Also distToT
        double distanceRemaining = Math.sqrt(Math.pow(delta_y, 2) + Math.pow(delta_x, 2)); // Euclidean Dist
        double accelerationTime = MAX_VEL / MAX_ACCEL;
        double accelerationDistance = 0.5 * MAX_ACCEL * Math.pow(accelerationTime, 2);
        double decelerationTime = MAX_VEL / MAX_ACCEL;
        double decelerationDistance = 0.5 * MAX_ACCEL * Math.pow(decelerationTime, 2);
        double cruiseDistance = distanceRemaining - accelerationDistance - decelerationDistance;
        double cruiseTime = cruiseDistance / MAX_VEL;
        double totalTime = accelerationTime + cruiseTime + decelerationTime;

        // Calculate the motion profile parameters
        double max_velocity = Math.min((double)MAX_VEL, (double)(distanceRemaining/2));
        double acceleration_time = max_velocity / MAX_ACCEL;
        double acceleration_distance = 0.5 * max_velocity * acceleration_time;
        double deceleration_distance = acceleration_distance;
        double cruise_distance = distance - acceleration_distance - deceleration_distance;
        double total_time = 2 * acceleration_time + (cruise_distance / max_velocity);


    }
    */

    public void DriveForAccuracy (XyhVector currentPosition, XyhVector startPosition, XyhVectorSA targetPosition, ElapsedTime startTime, double MAX_VEL, double MAX_ACCEL){
        // Current Positions
        double cPosX = currentPosition.getX();
        double cPosY = currentPosition.getY();
        double cPosT = currentPosition.getTheta();

        // Start Positions
        double startX = startPosition.getX();
        double startY = startPosition.getY();
        double startT = startPosition.getTheta();

        // Target Positions
        double targetX = targetPosition.getX();
        double targetY = targetPosition.getY();
        double targetT = targetPosition.getTheta();

        double delta_x = targetX - startX;
        double delta_y = targetY - startY;
        double distance = Math.sqrt(Math.pow(delta_x, 2) + Math.pow(delta_y, 2));

        // Create variables for MAX_VEL and MAX_ACCEL
        double max_velocity = MAX_VEL;
        double max_acceleration = MAX_ACCEL;

        // Calculate the motion profile parameters
        max_velocity = Math.min(MAX_VEL, distance / 2); // Limit max velocity
        double acceleration_time = max_velocity / max_acceleration;
        double acceleration_distance = 0.5 * max_velocity * max_acceleration;
        double deceleration_distance = acceleration_distance;
        double cruise_distance = distance - acceleration_distance - deceleration_distance;
        double total_time = 2 * acceleration_time + (cruise_distance / max_velocity);

        // The while loop is happening externally, so t<=total_time is substituted with if (drive.targetPositionReached) in main_loop:

        // Perform the motion profile

        double t = startTime.seconds();
        double current_velocity;
        double current_distance;

        if (t < acceleration_time) {
            current_velocity = max_acceleration * t;
            current_distance = 0.5 * max_acceleration * Math.pow(t, 2);
            telemetry.addData("State:", "Accelerating");
        }
        else if (t < (acceleration_time + (cruise_distance / max_velocity))){
            current_velocity = max_velocity;
            current_distance = acceleration_distance + (t - acceleration_time) * max_velocity;
            telemetry.addData("State:", "Cruising");
        }
        else {
            current_velocity =  max_velocity - max_acceleration * (
                    t - acceleration_time - cruise_distance / max_velocity);
            current_distance = acceleration_distance + cruise_distance + (
                    max_velocity * (t - acceleration_time - cruise_distance / max_velocity)) - (
                    0.5 * max_acceleration * (
                            Math.pow(t - acceleration_time - cruise_distance / max_velocity, 2)));
            telemetry.addData("State:", "Decelerating");
        }
        telemetry.update();

        double progress = current_distance / distance;
        double target_x = startX + progress * delta_x;
        double target_y = startY + progress * delta_y;

        double normalized_velocity_to_power = current_velocity / MAX_VEL;
        double normalized_x_power;
        double normalized_y_power;

        if (delta_x > delta_y){
            normalized_x_power = 1 * normalized_velocity_to_power;
            normalized_y_power = (delta_y / delta_x) * normalized_velocity_to_power;
        }
        else if (delta_y > delta_x){
            normalized_x_power = (delta_x / delta_y) * normalized_velocity_to_power;
            normalized_y_power = 1 * normalized_velocity_to_power;
        }
        else { // If delta_x == delta_y
            normalized_x_power = 1 * normalized_velocity_to_power;
            normalized_y_power = 1 * normalized_velocity_to_power;
        }
        double robot_movement_x_component = normalized_x_power;
        double robot_movement_y_component = normalized_y_power;
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
