package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Touch Sensor Test")
public class TouchTest extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveUntilTouch(0.3);
        sleep(1000);
        driveUntilTouch(-0.3);
    }
}
