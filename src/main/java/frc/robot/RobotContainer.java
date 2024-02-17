// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public final Camera GamePieceCam = new Camera("GamePiece");
  public final Camera frontTagCam = new Camera("frontTag", Constants.driveConstants.frontTagCamPose);
  public final Camera backTagCam = new Camera("backTag", Constants.driveConstants.backTagCamPose);
  public final Drive drive = new Drive(GamePieceCam, frontTagCam, backTagCam);
  public final Cobra cobra = new Cobra();
  public final Collector collector = new Collector();
  public final Climber climber = new Climber();
  public final LEDs leds = new LEDs();

  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private Boolean shootingInSpeaker = false;

  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  public final CommandXboxController coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(drive.driveWithJoysticks(
        driverController::getLeftY,
        driverController::getLeftX,
        driverController::getRightX,
        () -> false));

    climber.setDefaultCommand(climber.setCommand(coDriverController::getLeftY));

    cobra.setDefaultCommand(cobra.setSquisherAndIndexerCommand(() -> 0));
    collector.setDefaultCommand(collector.setCommand(() -> 0.1));
        
    // Configure the trigger bindings
    configureBindings();
    configureAutonomous();
  }

  private void configureAutonomous() {
    Command fourPiece = drive.loadChoreoTrajectory("4piece", false);

//    Command collectCommand = Commands.runOnce(() -> leds.setState(Constants.LEDStates.collecting)).
//            andThen(Commands.parallel(
////                    drive.driveToNote(cobra.isIndexerCurrentHigh()),
//                    cobra.cobraCollect(),
//                    collector.collect(cobra.isIndexerCurrentHigh()))).
//            andThen(Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing)));

//    autoChooser.addOption("four piece", fourPiece.alongWith(Commands.repeatingSequence(
//            collectCommand,
//            cobra.ShootSpeaker(drive::getPose))));
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
//    if (cobra.useCurrentControl) {
//      driverController.leftBumper().whileTrue(Commands.runOnce(() -> leds.setState(Constants.LEDStates.collecting)).
//              andThen(Commands.parallel(
//                drive.driveToNote(cobra.isIndexerCurrentHigh()),
//                cobra.cobraCollect(),
//                collector.collect(cobra.isIndexerCurrentHigh()))).
//              andThen(Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing))));
//    }
//    else {
//      driverController.leftBumper().whileTrue(Commands.runOnce(() -> leds.setState(Constants.LEDStates.collecting)).
//              andThen(
//              Commands.parallel(
//              drive.driveToNote(cobra.laserCan1Activated()),
//              cobra.cobraCollect(),
//              collector.collect(cobra.laserCan2Activated()))).
//              andThen(Commands.runOnce(() -> leds.setState(Constants.LEDStates.nothing))));
//    }

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

    driverController.leftBumper().onTrue(
            cobra.cobraCollect(collector.collect(cobra::laserCan2Activated)));

    driverController.rightBumper().onTrue(Commands.sequence(
                    cobra.setPivotPosCommand(() -> 1.575),
                    cobra.setSquisherVelCommand(() -> 100),
                    Commands.waitSeconds(0.5),
                    cobra.setIndexerCommand(() -> 0.5),
                    Commands.waitSeconds(0.5),
                    cobra.setSquisherAndIndexerCommand(() -> 0)));

    driverController.b().onTrue(Commands.sequence(
            cobra.setPivotPosCommand(() -> 1.35),
            cobra.setSquisherVelCommand(() -> Constants.cobraConstants.squisherShootSpeed),
            Commands.waitSeconds(0.5),
            cobra.setIndexerCommand(() -> 0.5),
            Commands.waitSeconds(0.5),
            cobra.setSquisherAndIndexerCommand(() -> 0)));

    driverController.a().onTrue(Commands.sequence(
            cobra.setSquisherVelCommand(() -> Constants.cobraConstants.squisherShootSpeed),
            cobra.setPivotPosCommand(() -> 1.365),
//            Commands.waitSeconds(0.6),
            cobra.setIndexerCommand(() -> 0.5),
            Commands.waitSeconds(0.5),
            cobra.setSquisherAndIndexerCommand(() -> 0)));

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
