/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robots.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class TalonSubsystem extends Subsystem {

  public class TalonConfiguration {

    private double feedForwardGain = 0;
    private double proportionalGain = 0;
    private double integralGain = 0;
    private double derivativeGain = 0;
    

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	  private int pidSlot = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
  	private int multiplePidLoopId = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    private int kTimeoutMs = 30;

    public TalonConfiguration() {

    }
    
    public TalonConfiguration(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain) {
      this.feedForwardGain = feedForwardGain;
      this.proportionalGain = proportionalGain;
      this.integralGain = integralGain;
      this.derivativeGain = derivativeGain;
    }

    public TalonConfiguration(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain, int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs) {
      this(feedForwardGain, proportionalGain, integralGain, derivativeGain);
      this.pidSlot = kSlotIdx;
      this.multiplePidLoopId = kPIDLoopIdx;
      this.kTimeoutMs = kTimeoutMs;
    }

    /**
     * @return the derivativeGain
     */
    public double getDerivativeGain() {
      return derivativeGain;
    }

    /**
     * @return the integralGain
     */
    public double getIntegralGain() {
      return integralGain;
    }

    /**
     * @return the proportionalGain
     */
    public double getProportionalGain() {
      return proportionalGain;
    }

    /**
     * @return the feedForwardGain
     */
    public double getFeedForwardGain() {
      return feedForwardGain;
    }

    public int getKTimeoutMs() {
      return kTimeoutMs;
    }

    public int getPidSlot() {
      return pidSlot;
    }

    public int getMultiplePidLoopId() {
      return multiplePidLoopId;
    }

  }

  public static void configureTalon(WPI_TalonSRX talon, TalonConfiguration config, FeedbackDevice feedbackDevice) {
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(feedbackDevice, config.getMultiplePidLoopId(), config.getKTimeoutMs());    
    talon.selectProfileSlot(config.getPidSlot(), config.getMultiplePidLoopId());
		talon.config_kF(config.getPidSlot(), config.getFeedForwardGain(), config.getKTimeoutMs());
		talon.config_kP(config.getPidSlot(), config.getProportionalGain(), config.getKTimeoutMs());
		talon.config_kI(config.getPidSlot(), config.getIntegralGain(), config.getKTimeoutMs());
    talon.config_kD(config.getPidSlot(), config.getDerivativeGain(), config.getKTimeoutMs());
  }

  public static void configureMotionMagicValues(WPI_TalonSRX talon, TalonConfiguration config, int velocityUnits, int accelerationUnits) {   
    talon.configMotionAcceleration(accelerationUnits, config.getKTimeoutMs());
    talon.configMotionCruiseVelocity(velocityUnits, config.getKTimeoutMs());
  }
  
  public static void printTalonOutputs(WPI_TalonSRX talon) {
        System.out.println("Sensor Vel:" + talon.getSelectedSensorVelocity());
        System.out.println("Sensor Pos:" + talon.getSelectedSensorPosition());
        System.out.println("Out %" + talon.getMotorOutputPercent());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
