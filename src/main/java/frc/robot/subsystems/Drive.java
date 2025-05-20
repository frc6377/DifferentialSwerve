package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;

public class Drive extends SubsystemBase {
  final double drivePercent = 0.5;
  TalonFX leftMotor;
  TalonFX rightMotor;
  TalonFXConfiguration config;

  public Drive() {
    leftMotor = new TalonFX(DriveConstants.leftMotorID);
    rightMotor = new TalonFX(DriveConstants.rightMotorID);

    config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  public Command rightMotorForward() {
    return Commands.startEnd(() -> rightMotor.set(drivePercent), () -> rightMotor.set(0));
  }

  public Command rightMotorBackward() {
    return startEnd(() -> rightMotor.set(-drivePercent), () -> rightMotor.set(0));
  }

  public Command leftMotorForward() {
    return Commands.startEnd(() -> leftMotor.set(drivePercent), () -> leftMotor.set(0));
  }

  public Command leftMotorBackward() {
    return Commands.startEnd(() -> leftMotor.set(-drivePercent), () -> leftMotor.set(0));
  }

  public Command motorsForward() {
    return Commands.startEnd(
        () -> {
          leftMotor.set(drivePercent);
          rightMotor.set(drivePercent);
        },
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        });
  }

  public Command motorsBackward() {
    return Commands.startEnd(
        () -> {
          leftMotor.set(-drivePercent);
          rightMotor.set(-drivePercent);
        },
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        });
  }

  public Command rightMotorForwardTriggerCommand(Supplier<Double> percent) {
    return Commands.startEnd(
        () -> rightMotor.set(Math.abs(percent.get()) * DriveConstants.kMaxSpeed),
        () -> rightMotor.set(0));
  }

  public Command rightMotorBackwardTriggerCommand(Supplier<Double> percent) {
    return Commands.startEnd(
        () -> rightMotor.set(-Math.abs(percent.get()) * DriveConstants.kMaxSpeed),
        () -> rightMotor.set(0));
  }

  public Command leftMotorForwardTriggerCommand(Supplier<Double> percent) {
    return Commands.startEnd(
        () -> leftMotor.set(Math.abs(percent.get()) * DriveConstants.kMaxSpeed),
        () -> leftMotor.set(0));
  }

  public Command leftMotorBackwardTriggerCommand(Supplier<Double> percent) {
    return Commands.startEnd(
        () -> leftMotor.set(-Math.abs(percent.get()) * DriveConstants.kMaxSpeed),
        () -> leftMotor.set(0));
  }
}
