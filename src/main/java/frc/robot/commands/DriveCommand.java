// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;

public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // RobotContainer.Subsystems.m_driveSubsystem.setCurrentLimits(60);
    SmartDashboard.putNumber("Gyro", Subsystems.m_driveSubsystem.getGyroYaw()%360);
    double forward = -Controllers.m_driverController.getLeftY();
    double rotate = -Controllers.m_driverController.getRightX();

    MathUtil.applyDeadband(forward, Constants.Controller.deadband);
    MathUtil.applyDeadband(rotate, Constants.Controller.deadband);

    forward = Math.signum(forward) * Math.pow(forward, 2);

    if (Controllers.m_driverController.getRightTriggerAxis()>0.05) {
    rotate = (rotate)/1;
    } else {
    rotate = (rotate)/1.45;
    }
    if(!RobotContainer.Controllers.m_driverController.getAButton() && !RobotContainer.Controllers.m_driverController.getXButton()){
       Subsystems.m_driveSubsystem.drive(forward, rotate);
    }
    // if (forward != 0 || rotate != 0) {
    
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
