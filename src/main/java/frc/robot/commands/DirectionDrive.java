// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DirectionDrive extends CommandBase {
  Drivetrain drivetrain;
  double tAngle;
  double speed;
  double distance;
  double tPower;
  double cAngle;
  double dAngle;
  double dir;
  /** Creates a new DirectionDrive. */
  public DirectionDrive(Drivetrain drivetrain, double tAngle, double speed, double distance){
    this.drivetrain = drivetrain;
    this.tAngle=tAngle;
    this.speed=speed;
    this.distance=distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){drivetrain.resetDEncoders();}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
   drivetrain.RobotMove(tAngle, speed, 0, false, false);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {drivetrain.stopMotors();drivetrain.resetDEncoders();}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getDEncoderPosition(0)>distance;
  }
}
