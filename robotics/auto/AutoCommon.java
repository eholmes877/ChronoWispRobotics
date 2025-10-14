package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

import edu.elon.robotics.base.RobotHardware;

/*
 * General autonomous methods.
 */
public class AutoCommon extends LinearOpMode {

    //how many degrees to adjust the requested degree angle by
    private final double ANGLE_OVERSHOOT = 2.0;

    //slow power of the motor for the final part of the turn
    private final double TURN_ENDING_POWER = 0.15;

    //number of degrees that will be done using the slow power
    private final double SLOW_DOWN_DEGREES = 15.0;

    private final double KP = 0.05;

    protected RobotHardware robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, true);
    }

    protected void driveForTime(double power, long milliseconds) {
        //start the motors to drive
        robot.startMove(power, 0, 0, 1);
        sleep(milliseconds);
        robot.startMove(0, 0, 0, 1);
    }

    protected void turnIMU(double degrees, double power) {
        //reset the yaw angle
        robot.imu.resetYaw();
        //start the motors turning the robot (based on power/degrees)
        if (degrees < 0) {
            robot.startMove(0.0, 0.0, Math.abs(power));
        }
        else {
            robot.startMove(0.0, 0.0, -Math.abs(power));
        }
        //wait until the heading of the robot matches the desired heading
        while (opModeIsActive() && Math.abs(robot.getHeading()) < (Math.abs(degrees) - ANGLE_OVERSHOOT)) {
            //System.out.println("[TURN_IMU] robot degrees: " + robot.getHeading());
            if (Math.abs(robot.getHeading()) - (Math.abs(degrees) - SLOW_DOWN_DEGREES) > 0.01) {
                //System.out.println("[TURN_IMU] is starting slow down");
                if (degrees < 0) {
                    robot.startMove(0.0, 0.0, TURN_ENDING_POWER);
                }
                else {
                    robot.startMove(0.0, 0.0, -TURN_ENDING_POWER);
                }
            }
        }

        //stop the motors
        robot.startMove(0.0, 0.0, 0.0);
    }

    protected void driveIMU(double cm, double power) {
        robot.resetDriveEncoders();
        robot.imu.resetYaw();
        robot.startMove(power, 0.0, 0.0);
        // moving forwards/backwards
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < Math.abs(robot.convertDistanceToTicks(cm))) {
            double strafeAdjustment = 0.0;
            double YawllDrive = robot.getHeading();
            strafeAdjustment = YawllDrive * KP;
            robot.motorAux.setPower(strafeAdjustment); // forward/backward movement, flip sign if opp. dir.
            System.out.println("TESTING: " + Math.abs(robot.motorLeft.getCurrentPosition()) + " with distance: " + Math.abs(robot.convertDistanceToTicks(cm)));
        }
        robot.startMove(0.0, 0.0, 0.0);
    }

    protected void turnAngle(double degrees, double maxPower) {
        double power = Math.abs(maxPower);
        robot.resetDriveEncoders();
        //robot.startMove(0.0, 0.0, power);
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < Math.abs(robot.convertDegreesToTicks(degrees))) {
            if (degrees < 0) {
                robot.startMove(0.0, 0.0, -(power));
            }
            else {
                robot.startMove(0.0, 0.0, power);
            }
        }
        robot.startMove(0.0, 0.0, 0.0);
    }

    protected void driveDistance(double cmForward, double cmSide, double maxPower) {
        DcMotor monitor = robot.motorLeft;
        double leftFwdMotorMovement = cmForward * robot.LR_TICKS_PER_FWD_CM;
        double rightFwdMotorMovement = -cmForward * robot.LR_TICKS_PER_FWD_CM;
        double auxSideMotorMovement = cmSide * robot.AUX_TICKS_PER_SIDE_CM;
        double leftSideMotorMovement = cmSide * robot.LR_TICKS_PER_SIDE_CM;
        double rightSideMotorMovement = cmSide * robot.LR_TICKS_PER_SIDE_CM;
        double leftMovement = leftFwdMotorMovement + leftSideMotorMovement;
        double rightMovement = rightFwdMotorMovement + rightSideMotorMovement;
        double maxDesiredTickValue = Math.max(auxSideMotorMovement, Math.max(leftMovement, Math.abs(rightMovement)));
        double distance = Math.sqrt(Math.pow(cmForward, 2) + Math.pow(cmSide, 2));
        double drivePower = maxPower * cmForward/distance;
        double strafePower = maxPower * cmSide/distance;

        System.out.println("TESTING: " + maxDesiredTickValue + "," + leftMovement + "," + rightMovement + "," + auxSideMotorMovement);

        if (Math.abs(maxDesiredTickValue - Math.abs(rightMovement)) < 0.01 ) {
            monitor = robot.motorRight;
            System.out.println("TESTING: right");
        }
        else if (Math.abs(maxDesiredTickValue - Math.abs(auxSideMotorMovement)) < 0.01) {
            monitor = robot.motorAux;
            System.out.println("TESTING: aux");
        } else {
            System.out.println("TESTING: left");
        }

        System.out.println("TESTING: " + maxDesiredTickValue);

        robot.resetDriveEncoders();
        robot.startMove(drivePower, strafePower, 0.0);
        while (opModeIsActive() && maxDesiredTickValue >= Math.abs(monitor.getCurrentPosition())) {
            //System.out.println("TESTING: " + Math.abs(monitor.getCurrentPosition()));
        }
        robot.startMove(0.0, 0.0, 0.0);
    }

    protected void driveUntilTouch(double power) {
        robot.resetDriveEncoders();
        robot.startMove(power, 0.0, 0.0);
        while(opModeIsActive()){
            if (robot.touchSensor.isPressed()) {
                robot.startMove(0.0, 0.0, 0.0);
                break;
            }

        }
        System.out.println("NEW FRUNK YEAH");
//        robot.motorLeft.getCurrentPosition();
//        while (robot.motorLeft.getCurrentPosition() > 0) {
//            robot.startMove(-power, 0.0, 0.0);
//        }
//        robot.startMove(0.0, 0.0, 0.0);

    }

    protected void driveToCalibrateLightSensor() {
        robot.maxBrightness = -1;
        robot.minBrightness = Integer.MAX_VALUE;
        robot.startMove(0.3, 0.0, 0.0);

        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < Math.abs(robot.convertDistanceToTicks(20))) {
            int lightIntensity = robot.colorSensor.alpha();
            if (lightIntensity < robot.minBrightness) {robot.minBrightness = lightIntensity;}
            else if (lightIntensity > robot.maxBrightness) {robot.maxBrightness = lightIntensity;}
        }
        System.out.println("MAX BRIGHTNESS: " + robot.maxBrightness);
        System.out.println("MIN BRIGHTNESS: " + robot.minBrightness);
        robot.startMove(0.0, 0.0, 0.0);

    }
}
