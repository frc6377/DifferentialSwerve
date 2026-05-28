import com.ctre.phoenix6.hardware.TalonFX;

public class Module {
    public final TalonFX topMotor;
    public final TalonFX bottomMotor;

    public Module(int driveMotorID, int steerMotorID) {
        topMotor = new TalonFX(driveMotorID);
        bottomMotor = new TalonFX(steerMotorID);
    }
}
