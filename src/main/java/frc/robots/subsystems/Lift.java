// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robots.subsystems;

import frc.robots.Robot;
import frc.robots.RobotMap;
import frc.robots.commands.*;
import frc.robots.subsystems.TalonSubsystem.TalonConfiguration;
import frc.robots.talonpidconstants.LiftTalonMotionMagicConstants;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Lift extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX liftTalon;
    private DoubleSolenoid liftActuator;
    private TalonConfiguration liftArmConfig;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Initialize your subsystem here
    public Lift() {

        liftTalon = new WPI_TalonSRX(RobotMap.LIFT_ARM_TALON_CAN_ID);
        liftTalon.configFactoryDefault();
        liftTalon.setSensorPhase(false);
        liftTalon.setInverted(false);
        // liftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        liftTalon.setNeutralMode(NeutralMode.Brake);
        // liftTalon.setSelectedSensorPosition(0, 0, 30);

        liftArmConfig = new TalonConfiguration(LiftTalonMotionMagicConstants.getLiftmotionmagicgains());
        TalonSubsystem.configureTalon(liftTalon, liftArmConfig, FeedbackDevice.CTRE_MagEncoder_Relative);
        TalonSubsystem.configureNominalAndPeakOutputs(liftTalon, liftArmConfig, 0, 0, 1, -1);
        TalonSubsystem.configureMotionMagicValues(liftTalon, liftArmConfig, LiftTalonMotionMagicConstants.getMotionmagiccruisevelocity(), LiftTalonMotionMagicConstants.getMotionmagicacceleration());
        TalonSubsystem.zeroSensor(liftTalon, liftArmConfig);    
        
        liftActuator = new DoubleSolenoid(0, 4, 5);
        liftActuator.set(Value.kForward);
        addChild("LiftActuator",liftActuator);
        

        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    
    @Override
    public void initDefaultCommand() {
        this.setDefaultCommand(new LiftUpDown());
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    public void setLiftPercentOutput(double output) {
        liftTalon.set(ControlMode.PercentOutput, output);
    }

    public void setLiftActuator(DoubleSolenoid.Value direction) {
        liftActuator.set(direction);
    }

    public WPI_TalonSRX getLiftTalon() {
        return this.liftTalon;
    }

    public boolean isLiftActuatorForward() {
        if(liftActuator.get() == Value.kForward) {
            return true;
        }
        return false;
    }

    public void printLiftTalonOutputs() {
        TalonSubsystem.printTalonOutputs(this.liftTalon);
    }

    public void putLiftTalonOutputsSmartDash() {
        TalonSubsystem.putTalonOutputsSmartDash(liftTalon);
    }

    public void setLiftPos1() {
        TalonSubsystem.setTalonMotionMagic(liftTalon, -LiftTalonMotionMagicConstants.getEncoderTargetValues()[0]);
    }

    public void setLiftPos2() {
        TalonSubsystem.setTalonMotionMagic(liftTalon, -LiftTalonMotionMagicConstants.getEncoderTargetValues()[1]);
    }

    public void setLiftPos3() {
        TalonSubsystem.setTalonMotionMagic(liftTalon, LiftTalonMotionMagicConstants.getEncoderTargetValues()[2]);
    }

    public void setLiftStowed() {
        TalonSubsystem.setTalonMotionMagic(liftTalon, LiftTalonMotionMagicConstants.getEncoderTargetValues()[3]);
    }

    public Value getLiftActuatorDirection() {
        return liftActuator.get();
    }

    public int getLiftEncoderPosition() {
        return liftTalon.getSelectedSensorPosition();
    }
}
