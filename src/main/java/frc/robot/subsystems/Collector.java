package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.collectorConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Collector extends SubsystemBase {

    private final CANSparkMax collectorMotor = new CANSparkMax(collectorConstants.collectorMotorID, CANSparkLowLevel.MotorType.kBrushless);

    public Collector() {

        collectorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        collectorMotor.setSmartCurrentLimit(collectorConstants.collectorMotorCurrentLimit);
        collectorMotor.setInverted(true);

        collectorMotor.getPIDController().setP(collectorConstants.collectorMotorP);
    }

    public void set(double speed) {
        collectorMotor.set(speed);
    }

    public void setVel(double speed) {
        collectorMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
    }

    public void stop() {
        collectorMotor.stopMotor();
    }

    public Command setCommand(DoubleSupplier speed) {
        return this.run(() -> set(speed.getAsDouble()));
    }

    public Command setVelCommand(DoubleSupplier speed) {
        return this.run(() -> setVel(speed.getAsDouble()));
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    public Command collect(BooleanSupplier untilWhen) {
        return setCommand(() -> -0.5).until(untilWhen);
    }
}
