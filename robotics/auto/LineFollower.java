package edu.elon.robotics.auto;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
@Autonomous (name = "Line Follower")
public class LineFollower extends AutoCommon {
    //how many degrees to adjust the requested degree angle by
    private final double ANGLE_OVERSHOOT = 2.0;

    //slow power of the motor for the final part of the turn
    private final double TURN_ENDING_POWER = 0.15;

    //number of degrees that will be done using the slow power
    private final double SLOW_DOWN_DEGREES = 15.0;

    // Kp = 0.6 / (robot.maxBrightness - robot.minBrightness + 0.0000001);

    private static double Kc = 0.04;
    private static double dT = 0.03;
    private static double Pc = 0.372222;

    private static double Kp = 0.000642;
    private static double Ki = 0.000085;  // (2.0 * Kp) * (dT / Pc)
    private static double Kd = 0.000592222; // (Kp * Pc) / (8*dT)


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        calibrateColorSensor();
        sleep(1500);
        //pController(0.3);
        pidController(0.2);
    }

    protected void pidController(double driveSpeed) { // following left side
        //ElapsedTime loopTimer = new ElapsedTime();
        robot.imu.resetYaw();
        robot.resetDriveEncoders();
        robot.startMove(driveSpeed, 0, 0);
        int averageLightIntensity = (robot.minBrightness + robot.maxBrightness) / 2;
        double sumError = 0;
        double previousError = 0;

        while (opModeIsActive()) {
            //loopTimer.reset();
            int lightIntensity = robot.colorSensor.alpha();
            int error = averageLightIntensity - lightIntensity;
            sumError = 0.8 * sumError + error;
            double turn = (Kp * error) + (Ki * sumError) + (Kd * (error - previousError));
            robot.startMove(driveSpeed, 0, -0.5*turn);
            previousError = error;
            //sleep(30 - Math.round(loopTimer.milliseconds()));
        }
    }


    protected void pController(double driveSpeed) { // following left side
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        robot.imu.resetYaw();
        robot.resetDriveEncoders();
        robot.startMove(driveSpeed, 0, 0);
        int averageLightIntensity = (robot.minBrightness + robot.maxBrightness) / 2;

        while (opModeIsActive()) {
            double lightIntensity = robot.colorSensor.alpha();
            System.out.println("P-CONTROL: " + timer.milliseconds() + ", " + lightIntensity);
            double Kp = 0.6 / (robot.maxBrightness - robot.minBrightness + 0.0000001);
            double lightIntensityError = lightIntensity - averageLightIntensity;
            robot.startMove(driveSpeed, 0, Kp * lightIntensityError);
        }
    }




    protected void calibrateColorSensor() {
        robot.maxBrightness = -1;
        robot.minBrightness = Integer.MAX_VALUE;
        int[] degreesList = {45, -90, 45};

        //System.out.println("BRIGHNESS degrees length " + degreesList.length);

        for (int i=0; i < degreesList.length; i++) {
            robot.imu.resetYaw();
            //System.out.println("BRIGHTNESS starting move: " + i);
            //System.out.println("BRIGHTNESS Degrees = " + degreesList[i]);
            if (degreesList[i] < 0) {
                robot.startMove(0.0, 0.0, Math.abs(0.3));
            } else {
                robot.startMove(0.0, 0.0, -Math.abs(0.3));
            }

            while (opModeIsActive() && Math.abs(robot.getHeading()) < (Math.abs(degreesList[i]) - ANGLE_OVERSHOOT)) { // maybe + ANGLE_OVERSHOOT? not syntax, just logic
                int lightIntensity = robot.colorSensor.alpha();
                //System.out.println("[BRIGHTNESS] robot degrees: " + robot.getHeading());
                if (Math.abs(robot.getHeading()) - (Math.abs(degreesList[i]) - SLOW_DOWN_DEGREES) > 0.01) {
                    if (lightIntensity < robot.minBrightness) {
                        robot.minBrightness = lightIntensity;
                    }
                    if (lightIntensity > robot.maxBrightness) {
                        robot.maxBrightness = lightIntensity;
                    }
                    //System.out.println("[TURN_IMU] is starting slow down");
                    if (degreesList[i] < 0) {
                        robot.startMove(0.0, 0.0, TURN_ENDING_POWER);
                    } else {
                        robot.startMove(0.0, 0.0, -TURN_ENDING_POWER);
                    }
                }
            }
        }
        System.out.println("MAX BRIGHTNESS: " + robot.maxBrightness);
        System.out.println("MIN BRIGHTNESS: " + robot.minBrightness);
        //stop the motors
        robot.startMove(0.0, 0.0, 0.0);
    }
}

