// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robots.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robots.Robot;
import frc.robots.RobotMap;
import frc.robots.subsystems.TalonSubsystem;
import frc.robots.talonpidconstants.LiftTalonMotionMagicConstants;

/**
 *
 */
public class LiftToLowPos extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double m_setpoint;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public LiftToLowPos() {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.lift);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {    
        // Robot.lift.setLiftActuator(Value.kReverse);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZE
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double targetPos = 0;
        // Robot.lift.setLiftPos2();
        System.out.println(LiftTalonMotionMagicConstants.getEncoderTargetValues()[1]);
        double liftStickAxis = 1.0 * Robot.oi.getController().getRawAxis(RobotMap.LIFT_STICK_Y_AXIS);
        // double adjustedStickOutput = (liftStickAxis + 1.0) / 2.0;
        if ((liftStickAxis) > -0.05) { liftStickAxis = 0;}
        System.out.println("Lift Stick Axis value: " + liftStickAxis);
        SmartDashboard.putNumber("Lift Stick Axis value", liftStickAxis);
        SmartDashboard.putNumber("Arm nominal output", Robot.lift.getNominalOutput());
        Robot.lift.putLiftTalonOutputsSmartDash();
        if(liftStickAxis < -0.05) {
            Robot.lift.resetNominalOutput();
            targetPos =  liftStickAxis * LiftTalonMotionMagicConstants.getkSensorUnitsPerRotation() * 3.0;
            System.out.println("Target Pos Value: " + targetPos);
            SmartDashboard.putNumber("Target Pos Value", targetPos);
            Robot.lift.setLiftPosWithJoystick(targetPos);
            Robot.lift.printLiftTalonOutputs();
        } 
        else {
            Robot.lift.configLiftNominalPercentOutput();
            targetPos = 0;
        }
       
        
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return (Robot.lift.getLiftEncoderPosition() >= LiftTalonMotionMagicConstants.getEncoderTargetValues()[1] - 20) &&
        Robot.lift.getLiftEncoderPosition() <= LiftTalonMotionMagicConstants.getEncoderTargetValues()[1] + 20;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ISFINISHED
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
