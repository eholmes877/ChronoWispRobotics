package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test IMU Driving")
public class TestIMUDriving extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveIMU(400, 0.3);
        telemetry.addData("Driving", 400 + " cm at " + 0.3 + " speed");
        telemetry.update();

        if (!opModeIsActive()) return;
        System.out.println("[DRIVE_IMU TESTING] requested: " + 400 + ", final: " + Math.abs(robot.motorLeft.getCurrentPosition()));
    }
}
