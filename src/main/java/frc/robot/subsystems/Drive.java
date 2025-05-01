package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;

public class Drive extends SubsystemBase {
  TalonFX leftMotor;
  TalonFX rightMotor;

  public Drive() {
    leftMotor = new TalonFX(DriveConstants.leftMotorID);
    rightMotor = new TalonFX(DriveConstants.rightMotorID);
  }

  public Command rightMotorForward() {
    return Commands.startEnd(() -> rightMotor.set(0.5), () -> rightMotor.set(0));
  }

  public Command rightMotorBackward() {
    return startEnd(() -> rightMotor.set(-0.5), () -> rightMotor.set(0));
  }

  public Command leftMotorForward() {
    return Commands.startEnd(() -> leftMotor.set(0.5), () -> leftMotor.set(0));
  }

  public Command leftMotorBackward() {
    return Commands.startEnd(() -> leftMotor.set(-0.5), () -> leftMotor.set(0));
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

  public Command leftMotorTriggerCommand(Supplier<Double> percent) {
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
