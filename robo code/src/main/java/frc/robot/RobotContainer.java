/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final OuttakeSubsystem m_outtakeDrive = new OuttakeSubsystem();

  //autonomous command, will spin robot in circle
  private final Command m_autoCommand =   new RunCommand(
    () -> m_robotDrive.tankDrive(0.2, -0.2),
    m_robotDrive);

  // The driver's controller
  static Joystick controller = new Joystick(DriveConstants.kControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands (will be run continously when nothing else is scheduled)
    
    m_robotDrive.setDefaultCommand(
      new ArcadeDrive(m_robotDrive)
    );
  }

  private void configureButtonBindings() {
    JoystickButton aButton = new JoystickButton(controller, ButtonConstants.kA);
    aButton.whenHeld(new OuttakeDrive(m_outtakeDrive, 0.1));

    JoystickButton bButton = new JoystickButton(controller, ButtonConstants.kB);
    bButton.whenHeld(new OuttakeDrive(m_outtakeDrive, -0.1));
  }

  public static double getMotorSpeed(int port) {
    // get a joystick axis
    return controller.getRawAxis(port);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //auto should spin in a circle
    return m_autoCommand;
  }
}