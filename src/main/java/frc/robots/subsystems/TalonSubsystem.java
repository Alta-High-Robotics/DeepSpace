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
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
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
    private int integralZone;
    private double peakOutputClosedLoop;

    public TalonPIDConfig(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain) {
      this.feedForwardGain = feedForwardGain;
      this.proportionalGain = proportionalGain;
      this.integralGain = integralGain;
      this.derivativeGain = derivativeGain;
    }

    public TalonPIDConfig(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain, int integralZone, double peakOutputClosedLoop)  {
      this(feedForwardGain, proportionalGain, integralGain, derivativeGain);
      this.integralZone = integralZone;
      this.peakOutputClosedLoop = peakOutputClosedLoop;
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

    public int getIntegralZone() {
      return integralZone;
    }

    public double getPeakOutputClosedLoop() {
      return peakOutputClosedLoop;
    }

  }
  
  public static class TalonConfiguration {

    
    private TalonPIDConfig closedLoopGains;
    private TalonPIDConfig auxClosedLoopGains;

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
    private int pidSlot = 0;
    
    private int auxPidSlot = 1;

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

    public TalonConfiguration(TalonPIDConfig closedLoopGains, TalonPIDConfig auxClosedLoopGains) {
      this(closedLoopGains);
      this.auxClosedLoopGains = auxClosedLoopGains;
    }

    public TalonConfiguration(TalonPIDConfig closedLoopGains, int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs) {
      this(closedLoopGains);
      this.pidSlot = kSlotIdx;
      this.multiplePidLoopId = kPIDLoopIdx;
      this.kTimeoutMs = kTimeoutMs;
    }

    public TalonConfiguration(TalonPIDConfig closedLoopGains, TalonPIDConfig auxClosedLoopGains, int kSlotIdx, int kPIDLoopIdx, int auxPIDid, int kTimeoutMs) {
      this(closedLoopGains, kSlotIdx, kPIDLoopIdx, kTimeoutMs);
      this.auxClosedLoopGains = auxClosedLoopGains;
      this.auxPidSlot = auxPIDid;
    }

    public TalonPIDConfig getClosedLoopGains() {
      return this.closedLoopGains;
    }

    public TalonPIDConfig getAuxClosedLoopGains() {
      return this.auxClosedLoopGains;
    }

    public int getKTimeoutMs() {
      return this.kTimeoutMs;
    }

    public int getPidSlot() {
      return this.pidSlot;
    }

    public int getAuxPidSlot() {
      return this.auxPidSlot;
    }

    public int getMultiplePidLoopId() {
      return this.multiplePidLoopId;
    }

  }

  public static void configureTalon(WPI_TalonSRX talon, TalonConfiguration config, FeedbackDevice feedbackDevice) {
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(feedbackDevice, config.getMultiplePidLoopId(), config.getKTimeoutMs());    
    talon.selectProfileSlot(config.getPidSlot(), config.getMultiplePidLoopId());
		configureTalonGains(talon, config);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, config.getKTimeoutMs());
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, config.getKTimeoutMs());
  }

  private static void configureTalonGains(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_kF(config.getPidSlot(), config.getClosedLoopGains().getFeedForwardGain(), config.getKTimeoutMs());
		talon.config_kP(config.getPidSlot(), config.getClosedLoopGains().getProportionalGain(), config.getKTimeoutMs());
		talon.config_kI(config.getPidSlot(), config.getClosedLoopGains().getIntegralGain(), config.getKTimeoutMs());
    talon.config_kD(config.getPidSlot(), config.getClosedLoopGains().getDerivativeGain(), config.getKTimeoutMs());
  }

  private static void configurePrimaryTalonGains(WPI_TalonSRX talon, TalonConfiguration config) {
    configureTalonGains(talon, config);
    // talon.config_IntegralZone(config.getPidSlot(), izone, timeoutMs)
  }


  private static void configureAuxTalonGains(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_kF(config.getAuxPidSlot(), config.getAuxClosedLoopGains().getFeedForwardGain(), config.getKTimeoutMs());
		talon.config_kP(config.getAuxPidSlot(), config.getAuxClosedLoopGains().getProportionalGain(), config.getKTimeoutMs());
		talon.config_kI(config.getAuxPidSlot(), config.getAuxClosedLoopGains().getIntegralGain(), config.getKTimeoutMs());
    talon.config_kD(config.getAuxPidSlot(), config.getAuxClosedLoopGains().getDerivativeGain(), config.getKTimeoutMs());
  }

  private static void configPrimaryIZoneAndClosedLoopSetting(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_IntegralZone(config.getPidSlot(), config.getClosedLoopGains().getIntegralZone(), config.getKTimeoutMs());
    talon.configClosedLoopPeakOutput(config.getPidSlot(), config.getClosedLoopGains().getPeakOutputClosedLoop(), config.getKTimeoutMs());
    talon.configAllowableClosedloopError(config.getPidSlot(), 0, config.getKTimeoutMs());
  }

  private static void configAuxIZoneAndClosedLoopSetting(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_IntegralZone(config.getAuxPidSlot(), config.getAuxClosedLoopGains().getIntegralZone(), config.getKTimeoutMs());
    talon.configClosedLoopPeakOutput(config.getAuxPidSlot(), config.getAuxClosedLoopGains().getPeakOutputClosedLoop(), config.getKTimeoutMs());
    talon.configAllowableClosedloopError(config.getAuxPidSlot(), 0, config.getKTimeoutMs());
  }


  public static void configureDriveTrainTalons(WPI_TalonSRX leftTalon, WPI_TalonSRX rightTalon, TalonConfiguration config, FeedbackDevice primaryDevice) {
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();
    leftTalon.configSelectedFeedbackSensor(primaryDevice, config.getMultiplePidLoopId(), config.getKTimeoutMs());
    rightTalon.configRemoteFeedbackFilter(leftTalon.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0, config.getKTimeoutMs());
    rightTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, config.getKTimeoutMs());				// Feedback Device of Remote Talon
    rightTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, config.getKTimeoutMs()); 
    rightTalon.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, config.getKTimeoutMs());
    rightTalon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, config.getKTimeoutMs());
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, config.getMultiplePidLoopId(), config.getKTimeoutMs());
    rightTalon.configSelectedFeedbackCoefficient(	0.5, config.getMultiplePidLoopId(), config.getKTimeoutMs());
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 1, config.getKTimeoutMs());
    rightTalon.configSelectedFeedbackCoefficient(1, 1, config.getKTimeoutMs());
    rightTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, config.getKTimeoutMs());
		rightTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, config.getKTimeoutMs());
		rightTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, config.getKTimeoutMs());
    rightTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, config.getKTimeoutMs());
    leftTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, config.getKTimeoutMs());

    rightTalon.configNeutralDeadband(0.001, config.getKTimeoutMs());
    leftTalon.configNeutralDeadband(0.001, config.getKTimeoutMs());
    configurePrimaryTalonGains(rightTalon, config);
    configPrimaryIZoneAndClosedLoopSetting(rightTalon, config);
    configureAuxTalonGains(rightTalon, config);
    configAuxIZoneAndClosedLoopSetting(rightTalon, config);

    int closedLoopTimeMs = 1;
    rightTalon.configClosedLoopPeriod(0, closedLoopTimeMs, config.getKTimeoutMs());
    rightTalon.configClosedLoopPeriod(1, closedLoopTimeMs, config.getKTimeoutMs());
    rightTalon.configAuxPIDPolarity(false, config.getKTimeoutMs());

  }

  public static void configureMotionMagicValues(WPI_TalonSRX talon, TalonConfiguration config, int velocityUnits, int accelerationUnits) {   
    talon.configMotionCruiseVelocity(velocityUnits, config.getKTimeoutMs());
    talon.configMotionAcceleration(accelerationUnits, config.getKTimeoutMs());
    
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
    double talonOutput = talon.getMotorOutputPercent();
    SmartDashboard.putNumber("pulseWidthPosition", pulseWidthWithoutOverflows);
    SmartDashboard.putNumber("selSenPos", selSenPos);
    SmartDashboard.putNumber("Talon output value: ", talonOutput);
}



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
