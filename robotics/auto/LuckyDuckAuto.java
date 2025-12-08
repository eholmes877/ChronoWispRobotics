package edu.elon.robotics.auto;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.base.ControlledServo;
import edu.elon.robotics.base.RobotHardware;

@Configurable
@Autonomous (name = "LDAuto")
public class LuckyDuckAuto extends AutoCommon {
    ControlledServo shoulder, elbow, gripper;
    private final double SHOULDER_UP = 0.45;
    private final double SHOULDER_DOWN = 0.15;
    private final double ELBOW_UP = 0.5;
    private final double ELBOW_DOWN = 1.0;
    private final double GRIPPER_OPEN = 0.75;
    private final double GRIPPER_CLOSED = 0.30;
    
    private double goalDistance = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, false);

        // wrap our servos within a ControlledServo
        shoulder = new ControlledServo(robot.servoShoulder, "shoulder", 0.5, 0.15, 0.85);
        elbow    = new ControlledServo(robot.servoElbow, "elbow", 0.5, 0.0, 1.0);
        gripper  = new ControlledServo(robot.servoGripper, "gripper", GRIPPER_OPEN, 0.3, 0.95);

        super.runOpMode();
        waitForStart();

        updateAll();

        findBoxSide();
        driveIMU(55, 0.3);
        shoulder.setPosition(SHOULDER_DOWN);
        elbow.setPosition(ELBOW_DOWN);
        gripper.setPosition(GRIPPER_OPEN);
        updateAll();
        sleep(500);
        gripper.setPosition(GRIPPER_CLOSED);
        updateAll();
        sleep(2000);
        shoulder.setPosition(SHOULDER_UP);
        elbow.setPosition(ELBOW_UP);
        updateAll();
        driveDistance(0.0, goalDistance, 0.3);
        driveIMU(3, 0.3);
        gripper.setPosition(GRIPPER_OPEN);
        updateAll();
        sleep(2000);

    }

    public void findBoxSide() {
        robot.maxBrightness = 480;
        robot.minBrightness = Integer.MAX_VALUE;
        robot.startMove(0.0, 0.3, 0.0);

        while (opModeIsActive() && Math.abs(robot.motorAux.getCurrentPosition()) < Math.abs(robot.convertDistanceToTicks(15))) {
            int lightIntensity = robot.colorSensor.alpha();
            if (lightIntensity < robot.minBrightness) {robot.minBrightness = lightIntensity;}
            else if (lightIntensity > robot.maxBrightness) {robot.maxBrightness = lightIntensity; goalDistance = 100; break;}
        }
        robot.startMove(0.0, 0.0, 0.0);
        sleep(500);
        robot.startMove(0.0, -0.3, 0.0);
        while (opModeIsActive() && Math.abs(robot.motorAux.getCurrentPosition()) < Math.abs(robot.convertDistanceToTicks(-30))) {
            int lightIntensity = robot.colorSensor.alpha();
            if (lightIntensity < robot.minBrightness) {robot.minBrightness = lightIntensity;}
            else if (lightIntensity > robot.maxBrightness) {robot.maxBrightness = lightIntensity; goalDistance = -50; break;}
            else if (robot.maxBrightness > 480) {break;}
        }
        robot.startMove(0.0, 0.0, 0.0);
        System.out.println("MAX BRIGHTNESS: " + robot.maxBrightness);
        System.out.println("MIN BRIGHTNESS: " + robot.minBrightness);
        sleep(2000);
    }

    public void updateAll() {
        shoulder.update();
        elbow.update();
        gripper.update();
    }
}
