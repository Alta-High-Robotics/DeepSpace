/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robots.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class TalonSubsystem extends Subsystem {

  public static class TalonPIDConfig {
    private double feedForwardGain = 0;
    private double proportionalGain = 0;
    private double integralGain = 0;
    private double derivativeGain = 0;

    public TalonPIDConfig(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain) {
      this.feedForwardGain = feedForwardGain;
      this.proportionalGain = proportionalGain;
      this.integralGain = integralGain;
      this.derivativeGain = derivativeGain;
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

  }

  public static class TalonConfiguration {

    
    private TalonPIDConfig closedLoopGains;

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

    public TalonConfiguration(TalonPIDConfig closedLoopGains) {
      this.closedLoopGains = closedLoopGains;
    }

    public TalonConfiguration(TalonPIDConfig closedLoopGains, int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs) {
      this.closedLoopGains = closedLoopGains;
      this.pidSlot = kSlotIdx;
      this.multiplePidLoopId = kPIDLoopIdx;
      this.kTimeoutMs = kTimeoutMs;
    }

    public TalonPIDConfig getClosedLoopGains() {
      return this.closedLoopGains;
    }

    public int getKTimeoutMs() {
      return this.kTimeoutMs;
    }

    public int getPidSlot() {
      return this.pidSlot;
    }

    public int getMultiplePidLoopId() {
      return this.multiplePidLoopId;
    }

  }

  public static void configureTalon(WPI_TalonSRX talon, TalonConfiguration config, FeedbackDevice feedbackDevice) {
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(feedbackDevice, config.getMultiplePidLoopId(), config.getKTimeoutMs());    
    talon.selectProfileSlot(config.getPidSlot(), config.getMultiplePidLoopId());
		talon.config_kF(config.getPidSlot(), config.getClosedLoopGains().getFeedForwardGain(), config.getKTimeoutMs());
		talon.config_kP(config.getPidSlot(), config.getClosedLoopGains().getProportionalGain(), config.getKTimeoutMs());
		talon.config_kI(config.getPidSlot(), config.getClosedLoopGains().getIntegralGain(), config.getKTimeoutMs());
    talon.config_kD(config.getPidSlot(), config.getClosedLoopGains().getDerivativeGain(), config.getKTimeoutMs());
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, config.getKTimeoutMs());
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, config.getKTimeoutMs());
  }

  public static void configureMotionMagicValues(WPI_TalonSRX talon, TalonConfiguration config, int velocityUnits, int accelerationUnits) {   
    talon.configMotionAcceleration(accelerationUnits, config.getKTimeoutMs());
    talon.configMotionCruiseVelocity(velocityUnits, config.getKTimeoutMs());
  }

  public static void configureNominalAndPeakOutputs(WPI_TalonSRX talon, TalonConfiguration config, double nomForward, double nomReverse, double peakForward, double peakReverse) {
    talon.configNominalOutputForward(nomForward, config.getKTimeoutMs());
		talon.configNominalOutputReverse(nomReverse, config.getKTimeoutMs());
		talon.configPeakOutputForward(peakForward, config.getKTimeoutMs());
    talon.configPeakOutputReverse(peakReverse, config.getKTimeoutMs());
  }

  public static void zeroSensor(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.setSelectedSensorPosition(0, config.getMultiplePidLoopId(), config.getKTimeoutMs());
  }
  
  public static void printTalonOutputs(WPI_TalonSRX talon) {
        System.out.println("Sensor Vel:" + talon.getSelectedSensorVelocity());
        System.out.println("Sensor Pos:" + talon.getSelectedSensorPosition());
        System.out.println("Out %" + talon.getMotorOutputPercent());
  }

  public static void setTalonMotionMagic(WPI_TalonSRX talon, double setpoint) {
    talon.set(ControlMode.MotionMagic, setpoint);
  }

  public static void putTalonOutputsSmartDash(WPI_TalonSRX talon) {
    int selSenPos = talon.getSelectedSensorPosition(0);
    int pulseWidthWithoutOverflows = talon.getSensorCollection().getPulseWidthPosition() & 0xFFF;

    SmartDashboard.putNumber("pulseWidthPosition", pulseWidthWithoutOverflows);
    SmartDashboard.putNumber("selSenPos", selSenPos);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
