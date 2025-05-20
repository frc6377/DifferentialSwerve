package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.Units.FeetPerSecond;

import java.util.function.Supplier;

public class Drive extends SubsystemBase {
    // The four swerve modules
    private final DiffSwerveModule frontLeft;
    private final DiffSwerveModule frontRight;
    private final DiffSwerveModule backLeft;
    private final DiffSwerveModule backRight;
    
    // Kinematics for the swerve drive
    private final SwerveDriveKinematics kinematics;
    
    public Drive() {
        // Create the four modules with their positions
        // Note: Module positions are relative to the center of the robot
        frontLeft = new DiffSwerveModule(
            0, 
            DriveConstants.kFrontLeftTopMotorID, 
            DriveConstants.kFrontLeftBottomMotorID,
            DriveConstants.kWheelBase / 2,  // x position
            DriveConstants.kTrackWidth / 2  // y position
        );
        
        frontRight = new DiffSwerveModule(
            1, 
            DriveConstants.kFrontRightTopMotorID, 
            DriveConstants.kFrontRightBottomMotorID,
            DriveConstants.kWheelBase / 2,    // x position
            -DriveConstants.kTrackWidth / 2   // y position
        );
        
        backLeft = new DiffSwerveModule(
            2, 
            DriveConstants.kBackLeftTopMotorID, 
            DriveConstants.kBackLeftBottomMotorID,
            -DriveConstants.kWheelBase / 2,   // x position
            DriveConstants.kTrackWidth / 2    // y position
        );
        
        backRight = new DiffSwerveModule(
            3, 
            DriveConstants.kBackRightTopMotorID, 
            DriveConstants.kBackRightBottomMotorID,
            -DriveConstants.kWheelBase / 2,   // x position
            -DriveConstants.kTrackWidth / 2   // y position
        );
        
        // Create the kinematics object with the module positions
        kinematics = new SwerveDriveKinematics(
            new Translation2d(frontLeft.getModuleX(), frontLeft.getModuleY()),
            new Translation2d(frontRight.getModuleX(), frontRight.getModuleY()),
            new Translation2d(backLeft.getModuleX(), backLeft.getModuleY()),
            new Translation2d(backRight.getModuleX(), backRight.getModuleY())
        );
    }
    
    /**
     * Drive the robot with specified speeds
     * @param xSpeed Forward/backward speed in meters per second
     * @param ySpeed Left/right speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     * @param fieldRelative Whether the provided speeds are relative to the field
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        // Calculate the swerve module states from the desired chassis movement
        SwerveModuleState[] moduleStates;
        
        if (fieldRelative) {
            // TODO: Add a gyro to get the robot rotation
            // For now, assume robot-relative
            moduleStates = kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, getRotation2d()
                )
            );
        } else {
            moduleStates = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
            );
        }
        
        // Normalize wheel speeds if any exceed the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates, DriveConstants.kMaxSpeed
        );
        
        // Set each module to its optimized state
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
    
    /**
     * Command to drive the robot with joystick inputs
     * @param xSupplier Forward/backward speed supplier (-1 to 1)
     * @param ySupplier Left/right speed supplier (-1 to 1)
     * @param rotSupplier Rotation speed supplier (-1 to 1)
     * @param fieldRelative Whether to use field-relative controls
     */
    public Command driveCommand(
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier,
            boolean fieldRelative) {
        
        return run(() -> {
            // Apply deadband and scaling to the inputs
            double xSpeed = DriveConstants.kMaxSpeed * 
                            applyDeadband(xSupplier.get(), DriveConstants.kDriveDeadband);
            
            double ySpeed = DriveConstants.kMaxSpeed * 
                            applyDeadband(ySupplier.get(), DriveConstants.kDriveDeadband);
            
            double rotSpeed = DriveConstants.kMaxSpeed * 
                              applyDeadband(rotSupplier.get(), DriveConstants.kDriveDeadband);
            
            drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
        });
    }
    
    /**
     * Apply a deadband to a joystick input
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        
        // Scale the output to be linear from deadband to 1
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }
    
    /**
     * Get the current rotation of the robot
     * In actual implementation, this would come from a gyro
     */
    private edu.wpi.first.math.geometry.Rotation2d getRotation2d() {
        // TODO: Replace with actual gyro reading
        return new edu.wpi.first.math.geometry.Rotation2d();
    }
    
    /**
     * Stop all motors
     */
    public Command stopCommand() {
        return runOnce(() -> {
            drive(0, 0, 0, false);
        });
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add telemetry or other periodic tasks here
    }
}