package edu.elon.robotics.base;

/*
 * Allow for two different kiwi drive ratios.
 *
 * @author K. Altmann and J. Hollingsworth
 */

public class KiwiDriveRatio {

    public double powerLeft = 0.0;
    public double powerRight = 0.0;
    public double powerAux = 0.0;

    private boolean isAuto = false;

    public KiwiDriveRatio(boolean isAuto) {
        this.isAuto = isAuto;
    }

    public void computeRatio(double drive, double strafe, double turn) {
        if (isAuto)
            computeRatioAuto(drive, strafe, turn);
        else
            computeRatioTeleop(drive, strafe, turn);

        normalize();
    }

    /**
     * computeRatioAuto
     * KiwiBot power ratios that maintain an equal amount of power for
     * driving, strafing, and turning. It's slower, but much easier to
     * work with in autonomous development.
     */
    private void computeRatioAuto(double drive, double strafe, double turn) {
        powerLeft  =  0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
        powerRight = -0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
        powerAux   =               - 2.0 * strafe / 3.0 + turn / 3.0;
    }

    /**
     * computeRatioTeleop
     * KiwiBot power ratios that maximize power for driving, strafing, and
     * turning. Turning is much faster than the others due to the angle
     * of the robot wheels.
     */
    private void computeRatioTeleop(double drive, double strafe, double turn) {
        powerLeft  = -0.5 * strafe + Math.sqrt(3.0) / 2.0 * drive + turn;
        powerRight = -0.5 * strafe - Math.sqrt(3.0) / 2.0 * drive + turn;
        powerAux   = strafe + turn;
    }

    /**
     * normalize()
     * Make sure all motor powers are between -1.0 and 1.0 by scaling all powers
     * by the largest power (normalization).
     */
    private void normalize() {
        double max = Math.max(1.0, Math.max(Math.abs(powerLeft), Math.max(Math.abs(powerRight), Math.abs(powerAux))));
        powerLeft /= max;
        powerRight /= max;
        powerAux /= max;
    }

}