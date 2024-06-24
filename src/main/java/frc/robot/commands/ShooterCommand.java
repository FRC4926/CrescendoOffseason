// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer.Controllers;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Robot;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  Timer rumble = new Timer();


  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.m_shooterSubsystem.changePassed(false);
    rumble.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance Sensor Voltage", Subsystems.m_shooterSubsystem.distanceSensor.getVoltage());
    //SmartDashboard.putBoolean("Distance Sensor Passed", Subsystems.m_shooterSubsystem.getPassed());

    //Rumble the operator controller when the target RPM is reached
    

    if(Subsystems.m_shooterSubsystem.distanceSensorTriggered()){
        Subsystems.m_shooterSubsystem.changePassed(true);
    }
    // if(RobotContainer.Controllers.m_operatorController.getRightY()>.2){
    //   Subsystems.m_shooterSubsystem.intake(Constants.Robot.intakeEffort);
    //   Subsystems.m_shooterSubsystem.convey(Constants.Robot.conveyorEffort);
    // }else if(RobotContainer.Controllers.m_operatorController.getRightY()<.2){
    //   Subsystems.m_shooterSubsystem.intake(-Constants.Robot.intakeEffort);
    //   Subsystems.m_shooterSubsystem.convey(Constants.Robot.conveyorEffort);
    // }

    if(Subsystems.m_armSubsystem.armMotor.getEncoder().getPosition() > (Constants.Robot.ampAngle-10)*180/Math.PI){
      //Change target RPM if it is an amp shot
       Subsystems.m_shooterSubsystem.setAmpMode(true);
    }else{
      //Set RPM
      Subsystems.m_shooterSubsystem.setAmpMode(false);
    }

    //reset color sensor in times of dire need
    if (Controllers.m_operatorController.getXButton())
    {
      Subsystems.m_shooterSubsystem.changePassed(Subsystems.m_shooterSubsystem.distanceSensorTriggered());
    }

    //Override the color sensor for full operator control
    if(Controllers.m_operatorController.getStartButton()){
      Subsystems.m_shooterSubsystem.setOverride(!Subsystems.m_shooterSubsystem.getOverride());

      //reset has passed in case you are switching back into override mode
      Subsystems.m_shooterSubsystem.changePassed(Subsystems.m_shooterSubsystem.distanceSensorTriggered());
    }

    if(Controllers.m_operatorController.getRightBumper()) 
    {

      if(Subsystems.m_shooterSubsystem.getAmpMode()){
        Subsystems.m_shooterSubsystem.zeroMotors();
        //Subsystems.m_shooterSubsystem.RPMShoot(-1000, -1000);
      }



      
      if(Subsystems.m_shooterSubsystem.isFinished(100))
      {
        //If the target is within 100 RPM of its target, switch to PID to hold it at it's target
        Subsystems.m_shooterSubsystem.RPMShoot(-Subsystems.m_shooterSubsystem.getTargetRPM(), -Subsystems.m_shooterSubsystem.getTargetRPM());  
      }else{
        //Provide maximum volatge otherwise
        Subsystems.m_shooterSubsystem.fullSend();
      }
    } else {
      //if(!Subsystems.m_shooterSubsystem.getOverride()){
        // if it's in overide, don't rev at steady state
        Subsystems.m_shooterSubsystem.RPMShoot(Constants.Robot.SteadySpeedRPM+500, Constants.Robot.SteadySpeedRPM+500);
      
      
    }

    
    if((Controllers.m_operatorController.getRightTriggerAxis()) > .2) 
    {
      //Shoot
      Subsystems.m_shooterSubsystem.convey(Constants.Robot.conveyorEffort);

      //Reset Color Sensor
      Subsystems.m_shooterSubsystem.changePassed(Subsystems.m_shooterSubsystem.distanceSensorTriggered());
    } else if(Controllers.m_operatorController.getLeftBumper()) 
    {
      //Outake at 1/2 speed
      Subsystems.m_shooterSubsystem.convey(-Constants.Robot.conveyorEffort/2);
      Subsystems.m_shooterSubsystem.intake(-Constants.Robot.intakeEffort/2);
      Subsystems.m_shooterSubsystem.distanceSensorTriggered();
    }
    else if(Controllers.m_operatorController.getLeftTriggerAxis()>.2) 
    {
      if(!Subsystems.m_shooterSubsystem.getOverride())
      {
        if (!Subsystems.m_shooterSubsystem.getPassed())
          {
            // Subsystems.m_shooterSubsystem.intake(Constants.Robot.intakeEffort);
            // Subsystems.m_shooterSubsystem.convey(Constants.Robot.conveyorEffort);
            if(Subsystems.m_shooterSubsystem.startStage()){
              Subsystems.m_shooterSubsystem.intake(-0.5);
              Subsystems.m_shooterSubsystem.convey(0.45);
            }else{
               Subsystems.m_shooterSubsystem.intake(Constants.Robot.autonIntakeEffort);
            Subsystems.m_shooterSubsystem.convey(Constants.Robot.conveyorEffort);
            }
           
          }
          else 
          {
            Subsystems.m_shooterSubsystem.intake(0);
            Subsystems.m_shooterSubsystem.convey(0);
          }
      }
      else 
      {
          //Intake at a lower speed and without color sensor dependance if overide mode is activated
          Subsystems.m_shooterSubsystem.intake(Constants.Robot.intakeEffort);
          Subsystems.m_shooterSubsystem.convey(Constants.Robot.conveyorEffort);
          Subsystems.m_shooterSubsystem.lowerMotor.setIdleMode(IdleMode.kBrake);
          Subsystems.m_shooterSubsystem.upperMotor.setIdleMode(IdleMode.kBrake);
      }
    }
    else{
      Subsystems.m_shooterSubsystem.intake(0);
      Subsystems.m_shooterSubsystem.convey(0);
      Subsystems.m_shooterSubsystem.lowerMotor.setIdleMode(IdleMode.kCoast);
      Subsystems.m_shooterSubsystem.upperMotor.setIdleMode(IdleMode.kCoast);
    }


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