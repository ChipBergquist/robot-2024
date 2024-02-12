// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Cobra;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Camera GamePieceCam = new Camera("GamePiece");
  public final Camera frontTagCam = new Camera("frontTag", Constants.driveConstants.frontTagCamPose);
  public final Camera backTagCam = new Camera("backTag", Constants.driveConstants.backTagCamPose);
  public final Drive drive = new Drive(GamePieceCam, frontTagCam, backTagCam);
  public final Cobra cobra = new Cobra();
  public final Collector collector = new Collector();
  public final Climber climber = new Climber();

  private Boolean shootingInSpeaker = false;

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final CommandXboxController coDriverController = 
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(drive.driveWithJoysticks(
        driverController::getLeftX, 
        driverController::getLeftY, 
        driverController::getRightX,
        () -> true));
        
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if (cobra.useCurrentControl) {
      driverController.leftBumper().whileTrue(Commands.parallel(
              drive.driveToNote(cobra.isIndexerCurrentHigh()),
              cobra.cobraCollect(),
              collector.collect(cobra.isIndexerCurrentHigh())));
    }
    else {
      driverController.leftBumper().whileTrue(Commands.parallel(
              drive.driveToNote(cobra.laserCan1Activated()),
              cobra.cobraCollect(),
              collector.collect(cobra.laserCan2Activated())));
    }

    driverController.rightBumper().whileTrue(shootingInSpeaker ?
                    Commands.parallel(
                            drive.run(() -> {
                                    ChassisSpeeds speeds = drive.getTargetSpeeds(
                                            driverController.getLeftX(),
                                            driverController.getLeftY(),
                                            drive.speakerShootAngle());
                                    drive.drive(
                                            new Translation2d(
                                                    speeds.vxMetersPerSecond,
                                                    speeds.vyMetersPerSecond),
                                            speeds.omegaRadiansPerSecond,
                                            true);
                            }),
                            cobra.ShootSpeaker(drive::getPose)) :
                            drive.driveToPose(new Pose2d()).andThen(cobra.scoreAmp()));

    coDriverController.x().onTrue(Commands.runOnce(() -> shootingInSpeaker = false));
    coDriverController.y().onTrue(Commands.runOnce(() -> shootingInSpeaker = true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
}
