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


import frc.robots.RobotMap;
import frc.robots.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.DoubleSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class HabClimb extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    
    private DoubleSolenoid frontHabClimbers;
    private DoubleSolenoid rearHabClimbers;
    private WPI_VictorSPX habClimberWheelOnLift;

    private VictorSP frontClimb;
    private VictorSP backClimb;

    

    

    public HabClimb() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // frontHabClimbers = new DoubleSolenoid(0, 6, 7);
        // addChild("FrontHabClimbers",frontHabClimbers);
        // frontHabClimbers.set(Value.kReverse);

        habClimberWheelOnLift = new WPI_VictorSPX(RobotMap.HAB_CLIMB_WHEEL_CAN_ID);
        
        
        // rearHabClimbers = new DoubleSolenoid(0, 4, 5);
        // addChild("RearHabClimbers",rearHabClimbers);
        // rearHabClimbers.set(Value.kForward);

        // frontClimb = new VictorSP(RobotMap.FRONT_MOTOR_CLIMB);
        // backClimb = new VictorSP(RobotMap.BACK_MOTOR_CLIMB);
        
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        // this.setDefaultCommand(new HabWheelOnLiftCommand());
        this.setDefaultCommand(new MotorClimb());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // public void setFrontSolenoid(DoubleSolenoid.Value direction) {
    //     frontHabClimbers.set(direction);
    // }

    // public void setBackSolenoid(DoubleSolenoid.Value direction) {
    //     rearHabClimbers.set(direction);
    // }

    // public void setFrontActuatorsUp() {
    //     setFrontSolenoid(Value.kReverse);
    // }

    // public void setFrontActuatorsDown() {
    //     setFrontSolenoid(Value.kForward);
    // }

    // public void setBackActuatorsUp() {
    //     setBackSolenoid(Value.kForward);
    // }

    // public void setBackActuatorsDown() {
    //     setBackSolenoid(Value.kReverse);
    // }

    // public boolean isFrontHabClimbDown() {
    //     if(frontHabClimbers.get() == Value.kReverse) {
    //         return true;
    //     }
    //     return false;
    // }

    // public boolean isBackHabClimbDown() {
    //     if(rearHabClimbers.get() == Value.kForward) {
    //         return true;
    //     }
    //     return false;
    // }

    public void setHabClimbWheelTalonPercentOutput(double output) {
        habClimberWheelOnLift.set(ControlMode.PercentOutput, output);
    }

    // public Value getFrontClimbersDirection() {
    //     return frontHabClimbers.get();
    // }

    // public Value getRearClimbersDirection() {
    //     return rearHabClimbers.get();
    // }

    public void climbUpFront() {
        frontClimb.set(0.25);
    }

    public void climbUpBack() {
        backClimb.set(-0.25);
    }

    public void climbDownFront() {
        frontClimb.set(-0.25);
    }

    public void climbDownBack() {
        backClimb.set(0.25);
    }

    public void setMotorClimbSpeed(double speed) {
        frontClimb.set(-speed);
        backClimb.set(speed);
    }

    public void setFrontMotorClimbSpeed(double speed) {
        frontClimb.set(1.0*speed);
    }

    public void idleFrontMotor() {
        frontClimb.set(-0.1);
    }

    public void setBackMotorClimbSpeed(double speed) {
        backClimb.set(1.0*speed);
    }


}

