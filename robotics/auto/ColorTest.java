package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Color Sensor Test")
public class ColorTest extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveToCalibrateLightSensor();

        //while (opModeIsActive()) {
        //    telemetry.addData("alpha", robot.colorSensor.alpha());
        //    telemetry.addData("red", robot.colorSensor.red());
        //    telemetry.addData("green", robot.colorSensor.green());
        //    telemetry.addData("blue", robot.colorSensor.blue());
        //    telemetry.update();
        //}

    }
}
