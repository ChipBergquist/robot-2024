// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
  
      public static final double ROBOT_MASS = 45.35924; // 32lbs * kg per pound
  
      // a matter var for limiting velocity
      public static final Matter CHASSIS = new Matter(new Translation3d(0, 0,
              Units.inchesToMeters(4)), ROBOT_MASS);

      public static final Transform3d frontTagCamPose = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
      public static final Transform3d backTagCamPose = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
  
      // loop time to use
      public static final double maxSpeed = Units.feetToMeters(14.5);

      public static final double WHEEL_DIAMETER = 3.036;
      public static final double DRIVE_GEAR_RATIO = 4.714285714285714;
      public static final double DRIVE_ENCODER_RESOLUTION = 1.0;

      public static final double STEERING_GEAR_RATIO = 1;
      public static final double STEERING_ENCODER_RESOLUTION = 1;

      public static final double autoCollectForwardVel = 0.65;// TODO: Tune me on real robot
      public static final double autoCollectTurnP = 0.041; // TODO: Tune me on real robot
      public static final double autoCollectMaxTurnVel = 0.3; // TODO: Tune me on real robot
  }

  public static class collectorConstants {
    public static final int collectorMotorID = 9;

    public static final double collectorMotorCurrentLimit = 40;
  }

  public static class climberConstants {
    public static final int climberMotor1ID = 10;
      public static final int climberMotor2ID = 11;

    public static final double lowerSoftLimit = 0;
    public static final double upperSoftLimit = 100;

    public static final double climberMotorCurrentLimit = 50;
  }

  public static class cobraConstants {
    public static final int pivotMotor1ID = 12;
    public static final int pivotMotor2ID = 13;
    public static final int squisherMotorID = 14;
    public static final int indexerMotorID = 15;

    public static final int laserCan1ID = 16;
    public static final int laserCan2ID = 17;
    public static final int pivotEncoderID = 18;

    public static final double rotationMotorCurrentLimit = 50;
    public static final double squisherMotorCurrentLimit = 40;
    public static final int indexerMotorCurrentLimit = 20;

    public static final double lowerRotationSoftLimit = 0;
    public static final double upperRotationSoftLimit = 100;

    public static final double pivotCollectAngle = 15;
    public static final double pivotGearRatio = 60;
    public static final double squisherGearRatio = 12;
    public static final double pivotAngleTolerance = 0.1;

    public static final double squisherSpeedTolerance = 0.5;

    public static final double laserCanDetectionTolerance = 5;
    public static final double squisherShootSpeed = 100;
    public static final double pivotAmpPos = 90;
  }

  public final class Field{
      public static final double blueSpeakerY = 5.612;
      public static final double redSpeakerY = 2.636;
      public static final double speakerWidth = 105;
      public static final double speakerX = 0.23;//16.412;
      public static final double speakerZ = 2.045; //height of opening
      public static final double blueAmpX = 0;
      public static final double redAmpX = 0;
      public static final double AmpY = 0;
  }
}
