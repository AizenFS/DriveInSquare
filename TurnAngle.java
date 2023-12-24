// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAngle extends PIDCommand {
  /** Creates a new TurnAngle. */
  private Drive drive;
  private double setPoint;
  
  public TurnAngle( Drive drive, double setpoint) {
    super(
        new PIDController(Constants.DriveConstants.kTurnP, Constants.DriveConstants.kTurnI,Constants.DriveConstants.kTurnD),
        drive::getHeading,
        setpoint,
        output -> drive.arcadeDrive(0, output),
        drive);

    this.drive=drive;
    this.setPoint = setpoint;
    getController().enableContinuousInput(-180, 180);
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  public void initialize(){
    drive.resetGyro();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drive.getHeading()>setPoint){
      return true;
    }
    return false;
  }
}
