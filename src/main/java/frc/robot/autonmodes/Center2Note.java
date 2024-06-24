// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.autoncommands.AutonConveyorCommand;
import frc.robot.commands.autoncommands.AutonIntakeCommand;
import frc.robot.commands.autoncommands.AutonShooterCommand;
import frc.robot.commands.autoncommands.AutonSlackinatorCommand;
import frc.robot.commands.autoncommands.AutonVisionCommand;

public class Center2Note {

  public static Command getCommand() {

    Trajectory trajectory1 = Subsystems.m_driveSubsystem.getTrajectory("Center2NoteForward");
    Trajectory trajectory2 = Subsystems.m_driveSubsystem.getTrajectory("Center2NoteBackward");

    return Commands.runOnce(() -> Subsystems.m_driveSubsystem.resetPose(trajectory1.getInitialPose()))
        .andThen(new AutonShooterCommand(Constants.Auton.subwooferTopRPM,Constants.Auton.subwooferBottomRPM).deadlineWith(new AutonSlackinatorCommand()))
        .andThen(Commands.waitSeconds(Constants.Auton.feedTime).deadlineWith(new AutonConveyorCommand()))
        .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.updateHasPassed()))
       // .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.intake(0)))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory1).alongWith(new AutonIntakeCommand()))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        //.deadlineWith(new AutonIntakeCommand())
        //.andThen(Commands.runOnce(()->Subsystems.m_intakeSubsystem.stop()))
        //.andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.intake(0)))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory2))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Commands.waitSeconds(Constants.Auton.feedTime).deadlineWith(new AutonConveyorCommand()))
        .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.zeroMotors())).alongWith(Commands.runOnce(()->Subsystems.m_shooterSubsystem.convey(0)));
  }
}
