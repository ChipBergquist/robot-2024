package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.collectorConstants;

public class Collector extends SubsystemBase {
    private TalonFX collectorMotor = new TalonFX(collectorConstants.collectorMotorID);

    public Collector() {
        // TalonFXConfigurator configs = collectorMotor.getConfigurator();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.CurrentLimits.SupplyCurrentLimit = collectorConstants.collectorMotorCurrentLimit;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.Audio.BeepOnConfig = true;
        
        collectorMotor.getConfigurator().apply(configs);
    }

    public void set(double speed) {
        collectorMotor.set(speed);
    }

    public void setVel(double speed) {
        collectorMotor.setControl(new VelocityVoltage(speed));
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
        return this.runOnce(() -> stop());
    }
}
