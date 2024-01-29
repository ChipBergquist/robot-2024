package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Camera;
import frc.robot.Constants.driveConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(),"swerve");
	SwerveDrive drive;

    Camera gamePieceCam;
    Camera frontTagCam;
    Camera backTagCam;

    private Boolean collected = false;

    public Drive(Camera gamePieceCam, Camera frontTagCam, Camera backTagCam) {
        this.gamePieceCam = gamePieceCam;
        this.frontTagCam = frontTagCam;
        this.backTagCam = backTagCam;

        // set the swerve telemetry's verbosity
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        // create the drivetrain from the config files
        try {
            double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(driveConstants.WHEEL_DIAMETER), 
                driveConstants.DRIVE_GEAR_RATIO,
                driveConstants.DRIVE_ENCODER_RESOLUTION);
            double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                driveConstants.STEERING_GEAR_RATIO, 
                driveConstants.STEERING_ENCODER_RESOLUTION);
            // SteeringConversionFactor = 360;
            drive = new SwerveParser(swerveJsonDir).createSwerveDrive(
                driveConstants.maxSpeed, 
                SteeringConversionFactor, 
                DriveConversionFactor);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void periodic() {
        updatePose();
        drive.updateOdometry();
    }

    private void updatePose() {
         Optional<EstimatedRobotPose> frontResult = frontTagCam.getPose(drive.getPose());
         if (frontResult.isPresent()) {
		 	EstimatedRobotPose camPose = frontResult.get();
		 	drive.addVisionMeasurement(
                 camPose.estimatedPose.toPose2d(),
                 camPose.timestampSeconds);
		 }
        
         Optional<EstimatedRobotPose> backResult = backTagCam.getPose(drive.getPose());
         if (backResult.isPresent()) {
		 	EstimatedRobotPose camPose = backResult.get();
		 	drive.addVisionMeasurement(
                 camPose.estimatedPose.toPose2d(),
                 camPose.timestampSeconds);
		 }
    }

    public Rotation2d getRoll() {
        return drive.getRoll();
    }

    public Rotation2d getPitch() {
        return drive.getPitch();
    }

    public Rotation2d getYaw() {
        return drive.getYaw();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		drive.drive(translation, rotation, fieldRelative, false);
	}

    public Command driveWithJoysticks(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotation, BooleanSupplier FieldRelative) {
        return this.run(() -> drive.drive(new Translation2d(
            -MathUtil.applyDeadband(Math.pow(vX.getAsDouble(),3), 0.04)*driveConstants.maxSpeed,
            -MathUtil.applyDeadband(Math.pow(vY.getAsDouble(), 3), 0.04)*driveConstants.maxSpeed),
            -MathUtil.applyDeadband(Math.pow(rotation.getAsDouble(), 3), 0.09) * drive.getSwerveController().config.maxAngularVelocity,
            FieldRelative.getAsBoolean(), false));
    }

    public void lock() {
        drive.lockPose();
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPitch()));
        // drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, true, false);
    }

    /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    drive.resetOdometry(initialHolonomicPose);
  }

  public Command driveToNote() {
      return this.runOnce(() -> collected = false).andThen(this.run(() -> {
          PhotonTrackedTarget target = gamePieceCam.getBestTarget();
          double angle = 0;
          if (target != null) {
              angle = target.getYaw();
              drive(new Translation2d(driveConstants.autoCollectForwardVel, 0),
                      Math.min(angle*-driveConstants.autoCollectTurnP, driveConstants.autoCollectMaxTurnVel),
                      false);
          }
          else {
              drive(new Translation2d(0, 0), 0, false);
              collected = true;
          }
      })).until(() -> collected);
  }

    public void defineAutoBuilder() {
		AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        drive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        drive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        drive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         new PIDConstants(5.0, 0.0, 0.0),
                                         // Translation PID constants
                                         new PIDConstants(drive.swerveController.config.headingPIDF.p,
                                                          drive.swerveController.config.headingPIDF.i,
                                                          drive.swerveController.config.headingPIDF.d),
                                         // Rotation PID constants
                                         driveConstants.maxSpeed,
                                         // Max module speed, in m/s
                                         drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to the furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
        this // Reference to this subsystem to set requirements
                                  );
	}

    public Command createTrajectory(String pathName, boolean setOdomToStart) {
		// Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
        resetOdometry(new Pose2d(path.getPoint(0).position, getYaw()));
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
	}
}
