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
import edu.wpi.first.wpilibj.command.Command;
import frc.robots.Robot;

/**
 *
 */
public class SeekRight extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    public SeekRight() {

        requires(Robot.driveTrain);
        requires(Robot.cameraSubsystem);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.cameraSubsystem.setTrackingMode();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    var data = Robot.cameraSubsystem.getData();
    if(data.targetExists == 0.0) {
      Robot.driveTrain.setArcadeDrive(0.0, 0.4);
      System.out.println("Seeking right");
    } else {
      // Positive is turn left, negative is turn right
      double minCommand = 0.1;
      double turnConstant = 0.02;
      double xOffset = 1.0 * Robot.cameraSubsystem.getData().xOffset;
      double headingError = 1.0 * xOffset;
      double turnToTargetRate = 0;

      double speedConstant = 0.7;

      if(xOffset > 1.0) {
        turnToTargetRate = turnConstant * headingError - minCommand;
      } else if(xOffset < 1.0) {
        turnToTargetRate = turnConstant * headingError + minCommand;
      }

      double speed = speedConstant - (0.105 * Robot.cameraSubsystem.getData().area);

      
      Robot.driveTrain.setArcadeDrive(speed, turnToTargetRate);
    }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.cameraSubsystem.getData().area >= 9.0;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.cameraSubsystem.setDriveCamMode();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
