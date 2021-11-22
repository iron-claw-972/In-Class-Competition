/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
public final class Constants {
  public static final class DriveConstants {
    //CHANGE PORTS
    public static final int kLeftMotorPort = 0;
    public static final int kRightMotorPort = 0;

    public static final int kControllerPort = 0;

    public static final int kLeftJoyAxis = 0;
    public static final int kRightJoyAxis = 0;
  }

  public static final class OuttakeConstants {
    //during testing, the port was 9-- CHECK PORTS
    public static final int kMotorSpinnerPort = 9;
  }

  public static final class ButtonConstants {
    //button A (1) is assigned to variable "kA"
    public static final int kA = 1;
    //button B (2) is assigned to variable "kB"
    public static final int kB = 2;
  }
}