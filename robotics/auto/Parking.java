package edu.elon.robotics.auto;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

enum spaceOpen { OPEN, NOT_OPEN }
@Configurable
@Autonomous (name = "Parking")
public class Parking extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        LineState state = LineState.NOT_ON_LINE;
        driveToCalibrateLightSensor();
        int minThreshold = (robot.maxBrightness - robot.minBrightness) / 5;
        int maxThreshold = (robot.maxBrightness - robot.minBrightness) * 4 / 5;
        robot.startMove(0.3, 0.0, 0.0);
        sleep(500);
        while (opModeIsActive()) {
            int lightIntensity = robot.colorSensor.alpha();
            if (lightIntensity > maxThreshold) {
                robot.startMove(0.0, 0.0, 0.0);
                sleep(2000);
                driveDistance(40.0, 0, 0.2);
                sleep(2000);
                turnIMU(90.0, 0.10);
                sleep(2000);
                break;
            }
        }

        double distanceThreshold = robot.distanceSensor.getDistance(DistanceUnit.CM);
        spaceOpen space = spaceOpen.NOT_OPEN;
        double initialLocation = 0;
        double finalLocation = 0;
        robot.startMove(0.2, 0.0, 0.0);
        double totalSpace = 0;
        System.out.println("DISTANCE THRESHOLD: " + distanceThreshold);
        while (opModeIsActive() & robot.colorSensor.alpha() < maxThreshold) {
            // log cat stuff
            double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
            if (distance - 40 > distanceThreshold && space == spaceOpen.NOT_OPEN) { //NOT_OPEN signifies it's beginning of a spot
                space = spaceOpen.OPEN;
                initialLocation = Math.abs((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM);
                System.out.println("INITIAL LOCATION: " + initialLocation + " INITIAL DISTANCE: " + distance);
            }
            if (distance - 20 < distanceThreshold && space == spaceOpen.OPEN) { // OPEN signifies it has been open, distance < threshold means it's the end of the line
                space = spaceOpen.NOT_OPEN;
                finalLocation = Math.abs((double)robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM);
                System.out.println("FINAL LOCATION: " + finalLocation + " FINAL DISTANCE: " + distance + " TOTAL LENGTH: " + (finalLocation - initialLocation));
            }
            if (Math.abs(finalLocation - initialLocation) >= 40 && space == spaceOpen.NOT_OPEN) {
                System.out.println("FINAL LOCATION: robot has an open space");
                robot.startMove(0.0, 0.0, 0.0);
                sleep(2000);
                driveDistance(-20, 0.0, 0.2);
                sleep(2000);
                while (robot.distanceSensor.getDistance(DistanceUnit.CM) > 20) {
                    robot.startMove(0.0, 0.2, 0.0);
                }
                robot.startMove(0.0, 0.0, 0.0);
                break;
            }
//            System.out.println("DISTANCE from wall: " + distance);
        }
    }
}

