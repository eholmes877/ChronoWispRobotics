package edu.elon.robotics.auto;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Configurable
@Autonomous(name ="Encoder Drive", group = "labs")
public class EncoderDrive extends AutoCommon{

    private static double driveSpeed = 0.3;
    private static int numTicks = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        robot.resetDriveEncoders();
        //robot.startMove(0.3, 0.0, 0.0);
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < numTicks) {
            robot.startMove(0.0, driveSpeed, 0.0);
        }
        robot.startMove(0.0, 0.0, 0.0);

        System.out.println("MYDATA: "
                    + robot.motorLeft.getCurrentPosition() + ","
                    + robot.motorRight.getCurrentPosition() + ","
                    + robot.motorAux.getCurrentPosition());
    }
}
