package edu.elon.robotics.base;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorAux;

    // control hub imu
    public IMU imu;

    // servos
    public Servo servoShoulder;
    public Servo servoElbow;
    public Servo servoGripper;

    //sensors
    public RevTouchSensor touchSensor;
    public ColorSensor colorSensor;
    public Rev2mDistanceSensor distanceSensor;

    public final KiwiDriveRatio ratio;

    public int maxBrightness;
    public int minBrightness;

    public final double TICKS_PER_ROTATION = 537.7;
    public final double WHEEL_CIRCUMFERENCE = 30.1593;
    public final double TICKS_PER_CM = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
    public final double LR_TICKS_PER_FWD_CM = TICKS_PER_CM * Math.sin(Math.PI / 3.0);
    public final double LR_TICKS_PER_SIDE_CM = TICKS_PER_CM * Math.cos(Math.PI / 3.0);
    public final double AUX_TICKS_PER_SIDE_CM = TICKS_PER_CM;
    public final double TURNING_DIAMETER = 25.25 / Math.cos(Math.toRadians(30));
    public final double TURNING_CIRCUMFERENCE = Math.PI * TURNING_DIAMETER;

    public int convertDistanceToTicks(double cm) {
        double distance = cm * TICKS_PER_CM;
        return (int) distance;
    }

    public int convertDegreesToTicks(double degrees) {
        double arc_length = degrees / 360.0 * TURNING_CIRCUMFERENCE;
        return convertDistanceToTicks(arc_length);
    }

    public RobotHardware(HardwareMap hardwareMap, boolean isAuto) {
        // define the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setPower(0);

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setPower(0);

        motorAux = hardwareMap.dcMotor.get("motorAux");
        motorAux.setDirection(DcMotor.Direction.REVERSE);
        motorAux.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorAux.setPower(0);

        // reset the drive encoders to zero
        resetDriveEncoders();

        // setup the motor ratio
        ratio = new KiwiDriveRatio(isAuto);

        /*
         * Define the orientation of the Control (and the IMU inside).
         */

        // the logo on the control hub is pointed up toward the sky
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        // the usb on the control hub is pointed up toward the forward
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // set this orientation
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);

        // code to get access to the servos
        servoShoulder = hardwareMap.get(Servo.class, "servoShoulder");
        servoElbow = hardwareMap.get(Servo.class, "servoElbow");
        servoGripper = hardwareMap.get(Servo.class, "servoGripper");

// now initialize the IMU with this mounting orientation
// this assumes the IMU to be in a REV Control Hub is named "imu"
        imu = hardwareMap.get(IMU.class, "imu");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


// define the current direction as 0
        imu.resetYaw();
    }

    public void resetDriveEncoders() {
        /*
         * This code resets the encoder values back to 0 for
         * each of the three drive motors.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAux.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getHeading() {
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    public void startMove(double drive, double strafe, double turn, double speedModifier) {
        /*
         * How much power should we apply to the left,
         * right, and aux motor?
         *
         * If all 3 motors apply the same power in the
         * same direction, the robot will turn in place.
         */
        ratio.computeRatio(drive, strafe, turn);

        /*
         * Limit the modifier.
         */
        speedModifier = Range.clip(speedModifier, 0.0, 1.0);

        /*
         * Apply the power to the motors.
         */
        motorLeft.setPower(ratio.powerLeft * speedModifier);
        motorRight.setPower(ratio.powerRight * speedModifier);
        motorAux.setPower(ratio.powerAux * speedModifier);
    }

    public void startMove(double drive, double strafe, double turn) {
        startMove(drive, strafe, turn, 1.0);
    }

}
