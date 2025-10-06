package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Pattern Driving", group="labs")
public class PatternDriving extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveDistance(150.0, 0.0, 0.3);
        sleep(200);
        driveDistance(-50.0, 0.0, 0.3);
        sleep(200);
        turnAngle(-90.0, 0.3);
        sleep(200);
        driveDistance(100.0, 0.0, 0.3);
        sleep(200);
        turnAngle(70.0, 0.3);
        sleep(200);
        driveDistance(-106.42, 0.0, 0.3);
        sleep(200);
        turnAngle(-250.0, 0.3);
        sleep(200);
        driveDistance(63.6, 0.0, 0.3);
        sleep(200);
        turnAngle(-90.0, 0.3);
    }
}
