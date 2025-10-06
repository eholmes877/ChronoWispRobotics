package edu.elon.robotics.teleop;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import edu.elon.robotics.base.RobotHardware;

@Configurable
@TeleOp(name = "Drive Robot", group = "TeleOp")
public class DriveRobot extends LinearOpMode {

    // represents the robot hardware (i.e., the robot).
    private RobotHardware robot;

    // provides continuous output while running
    private final TelemetryManager panelsTelemetry = Panels.getTelemetry();

    // control the base speed of the robot
    public static double speedModifier = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        // instantiate the robot (pass the hardware configuration)
        robot = new RobotHardware(hardwareMap, false);

        // block and wait until start is pressed
        waitForStart();

        // main loop
        while (opModeIsActive()){
            stickDriving();

            // show the encoder values for each motor
            panelsTelemetry.debug("encoder R: " + robot.motorRight.getCurrentPosition());
            panelsTelemetry.debug("encoder L: " + robot.motorLeft.getCurrentPosition());
            panelsTelemetry.debug("encoder A: " + robot.motorAux.getCurrentPosition());
            panelsTelemetry.update(telemetry);
        }

    }

    public void stickDriving() {
        /*
         * Read the gamepad joysticks and use that information
         * to drive the robot.
         */
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        /*
         * Telemetry shows up at the bottom of the
         * drive station. It's a good way to help
         * you debug your code.
         */
        panelsTelemetry.debug("drive: " + drive);
        panelsTelemetry.debug("strafe: " + strafe);
        panelsTelemetry.debug("turn: " +  turn);

        // make sure the speed modifier is between 0 and 1
        speedModifier = Range.clip(speedModifier, 0.0, 1.0);

        // call startMove to move the robot
        robot.startMove(drive, strafe, turn, speedModifier);
    }
}