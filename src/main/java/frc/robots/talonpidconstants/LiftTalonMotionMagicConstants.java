package frc.robots.talonpidconstants;

import frc.robots.subsystems.TalonSubsystem.TalonPIDConfig;

public class LiftTalonMotionMagicConstants {


    // Whether this loop is driving backwards or forwards: -1 for backwards, 1 for forwards
    private static final double forward = 1.0;
    private static final double backward = -1.0;

    private final double maxSensorVelocity  = 0;

    private static final int kSensorUnitsPerRotation = 4096;

    
    private static final double[] encoderTargetValues = {0, 0, 0, 0};

    

    private static final TalonPIDConfig liftMotionMagicGains = new TalonPIDConfig(0.0, 0.0, 0.0, 0.0);

    public static double[] getEncoderTargetValues() {
        return encoderTargetValues;
    }

    /**
	 * @return the liftmotionmagicgains
	 */
	public static TalonPIDConfig getLiftmotionmagicgains() {
		return liftMotionMagicGains;
	}

	/**
     * @return the maxSensorVelocity
     */
    public double getMaxSensorVelocity() {
        return maxSensorVelocity;
    }

    /**
     * @return the kSensorUnitsPerRotation
     */
    public static int getkSensorUnitsPerRotation() {
        return kSensorUnitsPerRotation;
    }

    /**
     * @return the backward
     */
    public static double getBackward() {
        return backward;
    }

    /**
     * @return the forward
     */
    public static double getForward() {
        return forward;
    }
}