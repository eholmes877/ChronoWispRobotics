package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


enum LineState { ON_LINE, NOT_ON_LINE }
@Autonomous (name="Line Counting")
public class CountingLines extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        LineState state = LineState.NOT_ON_LINE;
        robot.resetDriveEncoders();
        driveToCalibrateLightSensor(); // distance will be 20 cm
        sleep(500);
        robot.startMove(0.3, 0.0, 0.0);
        int minThreshold = (robot.maxBrightness - robot.minBrightness) / 5;
        int maxThreshold = (robot.maxBrightness - robot.minBrightness) * 4 / 5;
        //int tempMaxThreshold = maxThreshold;

        int numLines = 0;
        double lineWidth = 0;
        double lineStart = 0;
        double lineEnd = 0;
        System.out.println("FRUNK -- makes it to start of while loop");
        while (opModeIsActive()) {
            int lightIntensity = robot.colorSensor.alpha();
            System.out.println("Checking light intensity");
            //System.out.println("SENSITIVITY current rounded (to 1) distance: " + (Math.round(((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM) * 10.0) / 10.0));
           //if ((Math.round(((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM) * 10.0) / 10.0) % 0.1 == 0) {
            System.out.println("SENSITIVITY, " + (Math.round(((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM) * 10.0) / 10.0) + ", " + robot.colorSensor.alpha());

            if (lightIntensity > maxThreshold && state == LineState.NOT_ON_LINE) {
                state = LineState.ON_LINE;
                lineStart = (double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM;
                numLines += 1;


                //tempMaxThreshold = lightIntensity * 8/5;
            }
            if (lightIntensity < minThreshold && state == LineState.ON_LINE) {
                lineWidth = 0;
                state = LineState.NOT_ON_LINE;
                //tempMaxThreshold = maxThreshold;
                lineEnd = (double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM;
                lineWidth = lineEnd - lineStart;
                System.out.println("LINEWIDTH: Line #" + numLines + " = " + lineWidth + "cm");

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
//        driveDistance(distanceTraveled, 0, -0.3);
        while (opModeIsActive() && Math.abs((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM) < distanceTraveled) {

           robot.startMove(-0.3, 0.0, 0.0);

        }

        robot.startMove(0.0, 0.0, 0.0);
        sleep(5000);
    }
}
