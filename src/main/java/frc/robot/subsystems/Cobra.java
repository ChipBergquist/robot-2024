package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.cobraConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class Cobra extends SubsystemBase {
    private final TalonFX pivotMotor1 = new TalonFX(cobraConstants.pivotMotor1ID);
    private final TalonFX pivotMotor2 = new TalonFX(cobraConstants.pivotMotor2ID);
    private final TalonFX squisherMotor = new TalonFX(cobraConstants.squisherMotorID);
    private final CANSparkMax indexerMotor = new CANSparkMax(
            cobraConstants.indexerMotorID,
            MotorType.kBrushless);

    private final SparkPIDController indexerController;

    private final LaserCan laserCan1 = new LaserCan(cobraConstants.laserCan1ID);
    private final LaserCan laserCan2 = new LaserCan(cobraConstants.laserCan2ID);

    public Boolean useCurrentControl = false;

    private final LinearFilter indexerCurrent = LinearFilter.movingAverage(20);

    public Cobra() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        TalonFXConfiguration squisherConfigs = new TalonFXConfiguration();

        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cobraConstants.upperPivotSoftLimit;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = cobraConstants.lowerPivotSoftLimit;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.pivotMotorCurrentLimit;
        pivotConfigs.Audio.BeepOnConfig = true;
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfigs.Feedback.FeedbackRemoteSensorID = cobraConstants.pivotEncoderID;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = cobraConstants.pivotMotorAcceleration;
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = cobraConstants.pivotMotorVelocity;

        squisherConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        squisherConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        squisherConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.squisherMotorCurrentLimit;
        squisherConfigs.Audio.BeepOnConfig = true;
        squisherConfigs.MotionMagic.MotionMagicAcceleration = cobraConstants.squisherMotorAcceleration;
        squisherConfigs.MotionMagic.MotionMagicCruiseVelocity = cobraConstants.squisherMotorVelocity;

        pivotMotor1.getConfigurator().apply(pivotConfigs);
        pivotMotor2.getConfigurator().apply(pivotConfigs);
        pivotMotor2.setControl(new Follower(pivotMotor1.getDeviceID(), false));

        squisherMotor.getConfigurator().apply(squisherConfigs);

        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setInverted(false);
        indexerMotor.setSmartCurrentLimit(cobraConstants.indexerMotorCurrentLimit);

        indexerController = indexerMotor.getPIDController();

        try {
            laserCan1.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan2.setRangingMode(LaserCan.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
             DriverStation.reportError("one or both laser can devices have failed to configure", false);
             useCurrentControl = true;
        }
        if (laserCan1.getMeasurement().distance_mm < 0.1 || laserCan2.getMeasurement().distance_mm < 0.1) {
            useCurrentControl = true;
            DriverStation.reportError("one or both laser can devices have failed to give a correct measurement", false);
        }
    }

    @Override
    public void periodic() {
        indexerCurrent.calculate(indexerMotor.getOutputCurrent());
    }

    public BooleanSupplier laserCan2Activated() {
        return () -> laserCan2.getMeasurement().distance_mm < cobraConstants.laserCanDetectionTolerance;
    }

    public BooleanSupplier laserCan1Activated() {
        return () -> laserCan1.getMeasurement().distance_mm < cobraConstants.laserCanDetectionTolerance;
    }

    public BooleanSupplier noteInPosition() {
        return () ->    (laserCan2.getMeasurement().distance_mm < cobraConstants.laserCanDetectionTolerance) &&
                        !(laserCan1.getMeasurement().distance_mm < cobraConstants.laserCanDetectionTolerance);
    }

    public BooleanSupplier isIndexerCurrentHigh() {
        return () -> indexerCurrent.calculate(indexerMotor.getOutputCurrent()) > 20;
    }

    // rotation motor basic setters

    public void setPivotSpeed(double speed) {
        pivotMotor1.set(speed*cobraConstants.pivotGearRatio);
    }

    public void setPivotPos(double pos) {
        pivotMotor1.setControl(new PositionVoltage(pos*cobraConstants.pivotGearRatio));
    }

    public void stopPivot() {
        pivotMotor1.stopMotor();
    }

    // squisher motor basic setters

    public void setSquisher(double speed) {
        squisherMotor.set(speed);
    }

    public void setSquisherVel(double vel) {
        squisherMotor.setControl(new VelocityVoltage(vel));
    }

    public void stopSquisher() {
        squisherMotor.stopMotor();
    }

    public Boolean atSquisherSetpoint() {
        return squisherMotor.getClosedLoopError().getValueAsDouble() < cobraConstants.squisherSpeedTolerance;
    }

    // indexer motor basic setters

    public void setIndexer(double speed) {
        indexerMotor.set(speed);
    }

    public void setIndexerVel(double vel) {
        indexerController.setReference(vel, ControlType.kVelocity);
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
    }

    // COMMANDS //

    // rotation motor commands

    public Command setPivotCommand(DoubleSupplier speed) {
        return this.run(() -> setPivotSpeed(speed.getAsDouble()));
    }

    public Command setPivotPosCommand(DoubleSupplier pos) {
        return this.run(() -> setPivotPos(pos.getAsDouble())).
                until(this::atSquisherSetpoint);
    }

    public Command stopPivotCommand() {
        return this.runOnce(this::stopPivot);
    }

    // squisher motor commands

    public Command setSquisherCommand(DoubleSupplier speed) {
        return this.run(() -> setSquisher(speed.getAsDouble()));
    }

    public Command setsquisherVelCommand(DoubleSupplier vel) {
        return this.run(() -> setSquisherVel(vel.getAsDouble())).
                until(() -> squisherMotor.getClosedLoopError().getValueAsDouble() < cobraConstants.squisherSpeedTolerance);
    } 

    public Command stopSquisherCommand() {
        return this.runOnce(this::stopSquisher);
    }

    // indexer motor commands

    public Command setIndexerCommand(DoubleSupplier speed) {
        return this.run(() -> setIndexer(speed.getAsDouble()));
    }

    public Command setIndexerVelCommand(DoubleSupplier vel) {
        return this.run(() -> setIndexerVel(vel.getAsDouble()));
    }

    public Command stopIndexerCommand() {
        return this.runOnce(this::stopIndexer);
    }

    public Command cobraCollect() {
        if (useCurrentControl) {
            return setPivotPosCommand(() -> cobraConstants.pivotCollectAngle)
                    .andThen(Commands.parallel(
                            setSquisherCommand(() -> -0.5),
                            setIndexerCommand(() -> 0.3 * cobraConstants.indexerGearRatio)))
                    .until(isIndexerCurrentHigh())
                    .andThen(Commands.parallel(
                            setSquisherCommand(() -> -0.5),
                            setIndexerCommand(() -> -0.5 * cobraConstants.indexerGearRatio)))
                        .raceWith(Commands.waitSeconds(0.5));// wait half a second more to make sure the note is fully in the cobra
        }
        return setPivotPosCommand(() -> cobraConstants.pivotCollectAngle).
                andThen(Commands.parallel(
                        setSquisherCommand(() -> -0.5),
                        setIndexerCommand(() -> -0.5 * cobraConstants.indexerGearRatio)))
                .until(noteInPosition());
    }

    public Command ShootSpeaker(Supplier<Pose2d> robotPose) {
        return setsquisherVelCommand(() -> cobraConstants.squisherShootSpeed).alongWith(
                Commands.sequence(
                        setPivotPosCommand(() -> { // get the angle for the pivot
                            Translation2d speakerPose;
                            if (DriverStation.getAlliance().isPresent()) {
                                if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
                                    speakerPose = new Translation2d(0, Constants.Field.blueSpeakerY);
                                }
                                else{
                                    speakerPose = new Translation2d(0, Constants.Field.redSpeakerY);
                                }
                            }
                            else {
                                speakerPose = new Translation2d(0, 0);
                            }
                            double distanceFromSpeaker = speakerPose.getDistance(robotPose.get().getTranslation());
                            double height = Constants.Field.speakerZ - 0.48;

                            return Math.tan(height/distanceFromSpeaker);
                        }),
                        Commands.waitUntil(this::atSquisherSetpoint),
                        setIndexerCommand(() -> 0.5).deadlineWith(Commands.waitSeconds(0.5))));
    }

    public Command scoreAmp() {
        return setPivotPosCommand(() -> cobraConstants.pivotAmpPos)
                .andThen(setSquisherCommand(() -> 0.5))
                .alongWith(setIndexerCommand(() -> 0.5));
    }
}
