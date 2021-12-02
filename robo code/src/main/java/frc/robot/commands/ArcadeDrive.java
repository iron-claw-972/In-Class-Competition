/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.RobotContainer;
import frc.robot.config.Config;


public class ArcadeDrive extends CommandBase {
  private final DriveBase m_drive;

  public ArcadeDrive(DriveBase subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.arcadeDrive(
      RobotContainer.getMotorSpeed(Config.kRightJoyAxis), 
      RobotContainer.getMotorSpeed(Config.kLeftJoyAxis));
  }
} 