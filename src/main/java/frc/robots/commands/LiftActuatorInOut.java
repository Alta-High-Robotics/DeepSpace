/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robots.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robots.Robot;

public class LiftActuatorInOut extends TimedCommand {
  public LiftActuatorInOut(double timeout) {
    super(timeout);
    requires(Robot.lift);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first timed
  @Override
  protected void initialize() {
    // if(!Robot.lift.isLiftActuatorForward()) {
    //   Robot.lift.setLiftActuator(Value.kForward);
    // } else {
    //   Robot.lift.setLiftActuator(Value.kReverse);
    // }
    // System.out.println("Lift actuator working");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("Lift actuator direction: " + Robot.lift.getLiftActuatorDirection()); 
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
