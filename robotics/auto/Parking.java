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
        sleep(1000);
        int minThreshold = robot.minBrightness + (robot.maxBrightness - robot.minBrightness) / 5;
        int maxThreshold = robot.minBrightness +(robot.maxBrightness - robot.minBrightness) * 4 / 5;
        robot.startMove(0.15, 0.0, 0.0);
        sleep(2000);
        while (opModeIsActive()) {
            int lightIntensity = robot.colorSensor.alpha();
            if (lightIntensity > maxThreshold) {
                turnIMU(90.0, 0.10);
                sleep(2000);
                while (robot.distanceSensor.getDistance(DistanceUnit.CM) > 20) {
                    robot.startMove(0.0, 0.1, 0.0);
                }
                robot.startMove(0.0, 0.0, 0.0);
                sleep(1000);
                break;
            }
        }

        // find both spaces (both big enough) and the final positions associated. When the car has reached the final white line, determine smaller spot and move backward the position difference plus 20 cm

        double distanceThreshold = robot.distanceSensor.getDistance(DistanceUnit.CM);
        spaceOpen space = spaceOpen.NOT_OPEN;
        double initialLocation = 0;
        double finalLocation = 0;
        robot.startMove(0.1, 0.0, 0.0);
        double totalSpace = 0;
        System.out.println("DISTANCE THRESHOLD: " + distanceThreshold);
        double smallestSpotSize = Double.MAX_VALUE;
        double smallestSpotPosition = 0;
        double endLinePosition;
        while (opModeIsActive() && robot.colorSensor.alpha() < maxThreshold) {
            // log cat stuff
            double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
            if (distance - 40 > distanceThreshold && space == spaceOpen.NOT_OPEN) { //NOT_OPEN signifies it's beginning of a spot
                space = spaceOpen.OPEN;
                initialLocation = Math.abs((double) robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM);
                System.out.println("INITIAL LOCATION: " + initialLocation + " INITIAL DISTANCE: " + distance);
            }
            if (distance - 30 < distanceThreshold && space == spaceOpen.OPEN) { // OPEN signifies it has been open, distance < threshold means it's the end of the line
                space = spaceOpen.NOT_OPEN;
                finalLocation = Math.abs((double)robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM);
                System.out.println("FINAL LOCATION: " + finalLocation + " FINAL DISTANCE: " + distance + " TOTAL LENGTH: " + (finalLocation - initialLocation));
                if (finalLocation - initialLocation < smallestSpotSize && finalLocation - initialLocation > 35) { // this determines minimum parking size
                    smallestSpotSize = finalLocation - initialLocation;
                    System.out.println("DISTANCE Smallest Spot Size: " + smallestSpotSize);
                    smallestSpotPosition = robot.motorLeft.getCurrentPosition();
                    System.out.println("DISTANCE Smallest Spot Location: " + smallestSpotPosition);
                }
            }
            // removed logic of going directly to the spot
            if (robot.colorSensor.alpha() > maxThreshold) { // has reached the end
                robot.startMove(0.0, 0.0, 0.0);
                endLinePosition = robot.motorLeft.getCurrentPosition();
                break;
            }
        }
        double robotBackUp = -((robot.motorLeft.getCurrentPosition() / robot.TICKS_PER_CM) - (smallestSpotPosition / robot.TICKS_PER_CM)) - 20.0;
        if (smallestSpotSize < Double.MAX_VALUE) { // not sure if this comparison works, ask Joel if comparing
            System.out.print("DISTANCE Robot backs up " + robotBackUp);
            robot.startMove(robotBackUp, 0.0, 0.0);
            robot.startMove(0.0,0.0,0.0);
            double distanceFromWall = robot.distanceSensor.getDistance(DistanceUnit.CM) - 15;
            turnIMU(90.0, 0.10);
            driveIMU(distanceFromWall, 0.2);
            System.out.println("DISTANCE end spot location: " + robot.motorLeft.getCurrentPosition());
        }
        else {
            turnIMU(720.0, 0.7); //just some shenanigans, code still works
        }
    }
}
