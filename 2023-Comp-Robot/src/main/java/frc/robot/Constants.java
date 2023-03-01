// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  /* Speed Constants */

  public static final double speedLimit = 0.3;
  public static final double turretSpeed = .3;
  public static final double lowArmSpeed = .75;
  public static final double upArmSpeed = 1;

  /* Encoder Parameters */

  //Encoder counts per revelution
  private static final double unitsPerRev = 360;
  //Robot drive wheel diameter (Inches)
  private static final double whd = 6;
  //Encoder distance per pulse
  public static final double encPulse = Math.PI*whd/unitsPerRev;
  //Encoder prints
  public static int printLoops = 0;

  /* Sendable Chooser */
  public static final String defaultAuto = "Default";
  public static final String customAuto = "My Auto";
  final static SendableChooser<String> sendChooser = new SendableChooser<>();

  /* Limelight Stuff */
  public static double distanceFromLimelightToGoalInches;
  public static boolean havePlacedCone = false;

  public static double goalHeightInches = 0;

}
