package frc.robots.talonpidconstants;

public class DriveTrainMotionMagicConstants {

    // ALL MEASUREMENTS ARE IN INCHES
    private final double wheelDiameter = 7.65;
    private final double wheelRadius = wheelDiameter / 2;

    // We need to get the circumference of the wheel to determine how far our robot will travel in one rotation
    private final double wheelCircumference = Math.PI * Math.pow(wheelRadius, 2);

    // How far we want our robot to travel, in inches
    private final double distanceToTravelInInches = 15.0;

    // Calculate the number of rotations needed to travel how far we want to go
    private final double rotationsNeeded = distanceToTravelInInches / wheelCircumference;

    // Whether this loop is driving backwards or forwards: -1 for backwards, 1 for forwards
    private static final double forward = 1.0;
    private static final double backward = -1.0;

    private static final double maxSensorVelocity  = 0;

    private final int kSensorUnitsPerRotation = 4096;

    private final double encoderTargetValue = kSensorUnitsPerRotation * rotationsNeeded;

    public double getEncoderTargetValue() {
        return encoderTargetValue;
    }

    /**
     * @return the forward
     */
    public static double getForward() {
        return forward;
    }

    /**
     * @return the backward
     */
    public static double getBackward() {
        return backward;
    }

    /**
     * @return the maxsensorvelocity
     */
    public static double getMaxsensorvelocity() {
        return maxSensorVelocity;
    }
}