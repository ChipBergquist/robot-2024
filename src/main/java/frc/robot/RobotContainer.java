// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Camera GamePieceCam = new Camera("objectDetector");
  public final Camera frontTagCam = new Camera("front_arducam_OV9281", Constants.driveConstants.frontTagCamPose);
  public final Camera backTagCam = new Camera("back_arducam_OV9281", Constants.driveConstants.backTagCamPose);
  public final Drive drive = new Drive(GamePieceCam, frontTagCam, backTagCam);
  public final Cobra cobra = new Cobra();
  public final Collector collector = new Collector();
  public final Climber climber = new Climber();
  public final LEDs leds = new LEDs();

  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public final CommandXboxController coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  private Command scoreInAmp = Commands.sequence(
    Commands.runOnce(() -> leds.setState(Constants.LEDStates.amp)),
    cobra.setSquisherVelCommand(() -> 10),  // Make - squisherAmpShootSpeed
    cobra.setPivotPosCommand(() -> Constants.cobraConstants.pivotAmpAngle),
    cobra.setIndexerCommand(() -> 0.5), // CMB - Make Feed Out Speed. No Magic Numbers!

    Commands.waitSeconds(0.5),
    cobra.setSquisherAndIndexerCommand(() -> 0),  // CMB - Make Stop Speed. No Magic Numbers!
    cobra.setPivotCommand(() -> Constants.cobraConstants.pivotCollectAngle),
    Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing))
  );

  private Command scoreInSpeaker = Commands.sequence(
    Commands.runOnce(() -> leds.setState(Constants.LEDStates.speaker)),
    cobra.setSquisherVelCommand(() -> Constants.cobraConstants.squisherSpeakerShootSpeed),
    cobra.setPivotPosCommand(() -> Constants.cobraConstants.pivotSpeakerAngle),
    cobra.setIndexerCommand(() -> 0.5), // CMB - Make Feed Out Speed. No Magic Numbers!

    Commands.waitSeconds(0.5),
    cobra.setSquisherAndIndexerCommand(() -> 0), // CMB - Make Stop Speed. No Magic Numbers!
    cobra.setPivotCommand(() -> Constants.cobraConstants.pivotCollectAngle),
    Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing))
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Driver Joysticks, Both - Swerve Drive.
    drive.setDefaultCommand(drive.driveWithJoysticks(
        driverController::getLeftY,
        driverController::getLeftX,
        driverController::getRightX,
        () -> false));

    // Codriver Left Joystick  - Up and Down for Climber.
    climber.setDefaultCommand(climber.setCommand(coDriverController::getLeftY));

    // Default for Cobra's Squisher and Indexer are "stopped".
    // CMB - What and where is the defaut command for the Cobra's Pivot Arm?
    cobra.setDefaultCommand(cobra.setSquisherAndIndexerCommand(() -> 0));
    collector.setDefaultCommand(collector.setCommand(() -> 0.1));

    // leds.setDefaultCommand(Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing), leds));
        
    // CMB - Why do we put some default command here and not all?

    // Configure the trigger bindings
    configureBindings();
    configureAutonomous();
  }

// CMB - Iam completely ignoring Autonomous for now and focusing on Automation.
  private void configureAutonomous() {

    NamedCommands.registerCommand("shoot", Commands.sequence(
            cobra.shootSpeaker(drive::getPose),
            Commands.waitSeconds(0.5),
            cobra.setSquisherAndIndexerCommand(() -> 0)));

    NamedCommands.registerCommand("collect",
            cobra.cobraCollect(collector.collect(cobra::laserCan2Activated)));


    Command onePieceRun = drive.createTrajectory("1 piece", false);
    Command one5Piece = drive.createTrajectory("1.5 piece", false);
    Command threePiece = drive.createTrajectory("3 piece", false);

    autoChooser.addOption("1 piece", onePieceRun);
    autoChooser.setDefaultOption("3 piece", threePiece);
    autoChooser.addOption("1.5 piece", one5Piece);
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
    //    driverController.rightBumper().whileTrue(shootingInSpeaker ?
    //                    Commands.parallel(
    //                            drive.run(() -> {
    //                                    ChassisSpeeds speeds = drive.getTargetSpeeds(
    //                                            driverController.getLeftX(),
    //                                            driverController.getLeftY(),
    //                                            drive.speakerShootAngle());
    //                                    drive.drive(
    //                                            new Translation2d(
    //                                                    speeds.vxMetersPerSecond,
    //                                                    speeds.vyMetersPerSecond),
    //                                            speeds.omegaRadiansPerSecond,
    //                                            true);
    //                            }),
    //                            cobra.ShootSpeaker(drive::getPose)) :
    //                            drive.driveToPose(new Pose2d()).andThen(cobra.scoreAmp()));

    //    driverController.leftBumper().whileTrue(Commands.runOnce(() -> leds.setState(Constants.LEDStates.collecting)).
    //              andThen(
    //              Commands.parallel(
    //              cobra.cobraCollect(),
    //              collector.collect(cobra::laserCan2Activated))).
    //              andThen(Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing))));

    // Driver Left Bumper - Collect Note, Automated Driving.
    driverController.leftBumper().whileTrue(Commands.runOnce(() -> 
          leds.setState(Constants.LEDStates.collecting)).
          andThen(
                  Commands.parallel(
                          cobra.cobraCollect(collector.collect(cobra::laserCan2Activated)),
                          drive.driveToNote(cobra::laserCan2Activated))).
          andThen(Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing))));

    // Driver Right Bumper - Automated Amp Scoring, Manual Driving.
    driverController.rightBumper().onTrue(
      scoreInAmp
    );

    // Driver X - Collect Note, Manual Driving.
    driverController.x().onTrue(
      cobra.cobraCollect(
        collector.collect(cobra::laserCan2Activated)
        )
    );

    // Driver B - Automated Amp Scoring, Automated  Driving.
    driverController.b().whileTrue(
      drive.driveToPose(
        new Pose2d (
          new Translation2d(Constants.Field.ampX, Constants.Field.blueAmpY-1)
          , new Rotation2d(0)
        )
      )
      .andThen(
        scoreInAmp
      )
    );

    // Driver A - Automated Speaker Scoring, Manual Driving.
    driverController.a().onTrue(
      scoreInSpeaker
    );
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
