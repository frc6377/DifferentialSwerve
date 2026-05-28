// First, let's create a DiffSwerveModule class
package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class DiffSwerveModule {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final int moduleNumber;
    
    // Module position on the robot (for kinematics)
    private final double moduleX;
    private final double moduleY;
    
    public DiffSwerveModule(int moduleNumber, int topMotorID, int bottomMotorID, double moduleX, double moduleY) {
        this.moduleNumber = moduleNumber;
        this.moduleX = moduleX;
        this.moduleY = moduleY;
        
        // Initialize motors
        topMotor = new TalonFX(topMotorID);
        bottomMotor = new TalonFX(bottomMotorID);
        
        // Configure motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Apply configuration
        topMotor.getConfigurator().apply(config);
        bottomMotor.getConfigurator().apply(config);
    }
    
    /**
     * Sets the desired state for this swerve module.
     * @param desiredState The desired state with speed and angle
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid spinning more than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState, 
            new Rotation2d(getModuleAngle())
        );
        
        // Calculate the motor outputs based on the differential swerve math
        double[] motorOutputs = calculateDifferentialOutputs(
            optimizedState.speedMetersPerSecond,
            optimizedState.angle.getRadians()
        );
        
        // Set motor outputs
        topMotor.set(motorOutputs[0]);
        bottomMotor.set(motorOutputs[1]);
    }
    
    /**
     * Calculate the differential motor outputs required to achieve desired speed and angle
     * @param speedMetersPerSecond Target speed
     * @param angleRadians Target angle in radians
     * @return Array of [topMotorOutput, bottomMotorOutput]
     */
    private double[] calculateDifferentialOutputs(double speedMetersPerSecond, double angleRadians) {
        // This is the core differential swerve math
        // For differential swerve: 
        // - Sum of motor velocities = wheel velocity
        // - Difference of motor velocities = module rotation
        
        // Normalize speed to a percentage (-1 to 1)
        double normalizedSpeed = speedMetersPerSecond / ModuleConstants.kMaxModuleSpeedMetersPerSecond;
        
        // Calculate rotation component (how much the motors need to differ to achieve rotation)
        double rotationComponent = ModuleConstants.kRotationFactor; // This controls rotation speed
        
        // The key differential swerve equations:
        double topOutput = normalizedSpeed + rotationComponent * Math.sin(angleRadians);
        double bottomOutput = normalizedSpeed - rotationComponent * Math.sin(angleRadians);
        
        // Ensure outputs are within valid range
        return new double[] {
            Math.max(-1.0, Math.min(1.0, topOutput)),
            Math.max(-1.0, Math.min(1.0, bottomOutput))
        };
    }
    
    /**
     * Get the current angle of the module in radians
     */
    public double getModuleAngle() {
        // In a real implementation, you would use encoders to determine the actual angle
        // For differential swerve, this angle is calculated from the difference between motor positions
        
        double topPosition = topMotor.getPosition().getValueAsDouble();
        double bottomPosition = bottomMotor.getPosition().getValueAsDouble();
        
        return (topPosition - bottomPosition) * ModuleConstants.kDifferentialToAngleFactor;
    }
    
    /**
     * Get the current speed of the module in meters per second
     */
    public double getModuleVelocity() {
        // In a real implementation, you would use encoders to determine the actual velocity
        // For differential swerve, velocity is the sum of the motor velocities
        
        double topVelocity = topMotor.getVelocity().getValueAsDouble();
        double bottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
        
        return (topVelocity + bottomVelocity) * ModuleConstants.kDifferentialToVelocityFactor;
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getModuleVelocity(),
            new Rotation2d(getModuleAngle())
        );
    }
    
    public double getModuleX() {
        return moduleX;
    }
    
    public double getModuleY() {
        return moduleY;
    }
}