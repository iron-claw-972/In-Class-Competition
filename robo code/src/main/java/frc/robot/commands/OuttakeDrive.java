/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeDrive extends CommandBase {
  private final OuttakeSubsystem m_outtake;

  public OuttakeDrive(OuttakeSubsystem subsystem) {
      m_outtake = subsystem;
      addRequirements(m_outtake);
  }
  }

  @Override
  public void execute() {
    m_outtake.run(.1);
  }

  @Override
  public void end(boolean interrupted){
    m_outtake.run(0.0);
  }
}