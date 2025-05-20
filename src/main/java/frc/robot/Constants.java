package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {

  public static class OperatorConstants {
  public static final int kDriverControllerPort = 0;
}
    public static final class DriveConstants {
        // Motor IDs for each module
        public static final int kFrontLeftTopMotorID = 1;
        public static final int kFrontLeftBottomMotorID = 2;
        public static final int kFrontRightTopMotorID = 3;
        public static final int kFrontRightBottomMotorID = 4;
        public static final int kBackLeftTopMotorID = 5;
        public static final int kBackLeftBottomMotorID = 6;
        public static final int kBackRightTopMotorID = 7;
        public static final int kBackRightBottomMotorID = 8;
        
        // Robot physical dimensions
        public static final double kTrackWidth = 0.5;  // Distance between left and right wheels in meters
        public static final double kWheelBase = 0.7;   // Distance between front and back wheels in meters
        
        // Speed limits
        // public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(3.0);
        // public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(Math.PI);

        public static final double kMaxSpeed = 4.0; // 4 m/s
        public static final double kMaxAngularSpeed = Math.PI; // 2π rad/s
        
        // Joystick deadband
        public static final double kDriveDeadband = 0.05;
    }
    
    public static final class ModuleConstants {
        // Maximum speed of an individual module
        public static final double kMaxModuleSpeedMetersPerSecond = 4.0;
        
        // Conversion factors for the differential swerve module
        // These will depend on your specific gear ratios and wheel size
        public static final double kDifferentialToAngleFactor = 0.01;  // Convert differential position to angle
        public static final double kDifferentialToVelocityFactor = 0.01;  // Convert sum to velocity
        
        // Factor to adjust rotation speed of modules
        public static final double kRotationFactor = 0.5;
    }
}