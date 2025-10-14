package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Line Counting")
public class CountingLines extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        robot.resetDriveEncoders();
        driveToCalibrateLightSensor(); // distance will be 20 cm
        robot.startMove(0.3, 0.0, 0.0);
        int minThreshold = (robot.maxBrightness - robot.minBrightness) / 5;
        int maxThreshold = (robot.maxBrightness - robot.minBrightness) * 4 / 5;
        int tempMaxThreshold = maxThreshold;
        int numLines = 0;
        System.out.println("FRUNK -- makes it to start of while loop");
        while (opModeIsActive()) {
            int lightIntensity = robot.colorSensor.alpha();
            System.out.println("Checking light intensity");
            if (lightIntensity > tempMaxThreshold) {
                numLines += 1;
                tempMaxThreshold = lightIntensity * 8/5;
            }
            if (lightIntensity < minThreshold) {
                tempMaxThreshold = maxThreshold;
            }
            if (robot.touchSensor.isPressed()) {
                robot.startMove(0.0, 0.0, 0.0);
                break;
            }

        }

        double distanceTraveled = (double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM;
        System.out.println("NEW FRUNK YEAH! Distance traveled (in stupid ticks): " + distanceTraveled);
        telemetry.addData("Distance (cm)", distanceTraveled);
        telemetry.addData("Lines", numLines);
        telemetry.update();
        sleep(3000);
        robot.resetDriveEncoders();
        while (opModeIsActive() && Math.abs((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM) < distanceTraveled) {

            robot.startMove(-0.3, 0.0, 0.0);
        }

        robot.startMove(0.0, 0.0, 0.0);
        sleep(5000);
    }
}
