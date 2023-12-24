package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

/** A command that will turn the robot to the specified angle. */
public class DriveDis extends PIDCommand {
  private Drive drive;
  private double setPoint;
  
  public DriveDis(Drive drive, double setpoint) {
    super(
        new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD),
        drive::getBothEncoders,
        setpoint,
        output -> drive.arcadeDrive(output, 0),
        drive);

    this.drive = drive;
    this.setPoint = setpoint;
    getController().enableContinuousInput(-3, 3);

    getController()
        .setTolerance(DriveConstants.kDriveToToleranceMeters, DriveConstants.kDriveRateToleranceMetersPerS);
    getController().reset();
    

  }

  @Override
  public void initialize(){
    drive.resetEncoders();
  }

  
  @Override
  public boolean isFinished() {
     if(drive.getBothEncoders()> setPoint*0.9){
      return true;
     }
     return false;
    
  }
}