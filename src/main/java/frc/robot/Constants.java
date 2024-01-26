// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 0;
  }

  public static class driveConstants {
      public static final double balanceP = 0.05;
      public static final double balanceI = 0;
      public static final double balanceD = 0;
  
      public static final double ROBOT_MASS = 45.35924; // 32lbs * kg per pound
  
      // a matter var for limiting velocity
      public static final Matter CHASSIS = new Matter(new Translation3d(0, 0,
              Units.inchesToMeters(4)), ROBOT_MASS);
  
      // loop time to use
      public static final double LOOP_TIME = 0.13;
      public static final double maxSpeed = 4;

      public static final double WHEEL_DIAMETER = 3.36;
      public static final double DRIVE_GEAR_RATIO = 4.714285714285714;
      public static final double DRIVE_ENCODER_RESOLUTION = 1.0;

      public static final double STEERING_GEAR_RATIO = 1;
      public static final double STEERING_ENCODER_RESOLUTION = 1;
  }

  public static class collectorConstants {
    public static final int collectorMotorID = 9;

    public static final double collectorMotorCurrentLimit = 40;
  }

  public static class climberConstants {
    public static final int climberMotorID = 10;

    public static final double lowerSoftLimit = 0;
    public static final double upperSoftLimit = 100;

    public static final double climberMotorCurrentLimit = 50;
  }

  public static class cobraConstants {
    public static final int pivotMotorID = 11;
    public static final int squisherMotorID = 12;
    public static final int indexerMotorID = 13;

    public static final int laserCan1ID = 14;
    public static final int laserCan2ID = 15;

    public static final int pivotEncoderID = 0;

    public static final double pivotP = 0.01;
    public static final double pivotI = 0;
    public static final double pivotD = 0;

    public static final double rotationMotorCurrentLimit = 50;
    public static final double squisherMotorCurrentLimit = 40;
    public static final int indexerMotorCurrentLimit = 20;

    public static final double lowerRotationSoftLimit = 0;
    public static final double upperRotationSoftLimit = 100;

    public static final double pivotCollectAngle = 15;
  }
}
