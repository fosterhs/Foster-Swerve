package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

final class SwerveModule {
  private static final double wheelCirc = 4.0*0.0254*Math.PI; // Circumference of the wheel. Unit: meters
  private static final double falconEncoderRes = 2048.0;
  private static final double turnGearRatio = 150.0/7.0;
  private static final double driveGearRatio = 57.0/7.0;
  private static final double currentLimit = 40.0;
  private static final double wheelEncoderRes = 4096.0;

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  private final AnalogEncoder wheelEncoder;

  // Keeps track wraparounds when the wheel angle crosses 180 degrees 
  public double goalAng = 0.0;
  private double prevGoalAng = 0.0;
  private double wraparoundOffset = 0.0;

  public SwerveModule(int turnID, int driveID, int encoderID, boolean invertDrive) {
    driveMotor = new WPI_TalonFX(driveID);
    turnMotor = new WPI_TalonFX(turnID);
    wheelEncoder = new AnalogEncoder(encoderID);

    driveMotor.configFactoryDefault();
    turnMotor.configFactoryDefault();
    
    // Limits current draw for each motor to 40 amps
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = currentLimit;
    config.supplyCurrLimit.triggerThresholdTime = 0.1;
    config.supplyCurrLimit.currentLimit = currentLimit;

    turnMotor.configAllSettings(config);
    turnMotor.setSelectedSensorPosition(0);
    turnMotor.setNeutralMode(NeutralMode.Brake);
    
    // Motion Magic parameters for the turning motor
    double kI_turn = 0.001;
    turnMotor.config_kP(0, 0.4);
    turnMotor.config_kI(0, kI_turn);
    turnMotor.config_kD(0, 3.0);
    turnMotor.configMotionAcceleration(100000);
    turnMotor.configMotionCruiseVelocity(200000);
    turnMotor.configAllowableClosedloopError(0, 20);
    turnMotor.configMaxIntegralAccumulator(0, 0.8*1023/kI_turn);

    driveMotor.configAllSettings(config);
    driveMotor.setSelectedSensorPosition(0);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    
    // Velocity control parameters for the drive motor
    double kI_drive = 0.0003;
    driveMotor.config_kP(0, 0.04);
    driveMotor.config_kI(0, kI_drive);
    driveMotor.config_kD(0, 1.0);
    driveMotor.config_kF(0, 0.0447);
    driveMotor.configMaxIntegralAccumulator(0, 0.8*1023.0/kI_drive);

    if (invertDrive) {
      driveMotor.setInverted(true);
    }
    turnMotor.setInverted(true);
  }
  
  // Sets the swerve module to the given state.
  public final void setState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
    double goalVel = optimizedState.speedMetersPerSecond;
    goalAng = optimizedState.angle.getDegrees();
    
    // Handles wraparounds that occur at 180/-180 degrees
    if (goalAng-prevGoalAng > 240.0) {  // -180 -> 180 wraparound 
      wraparoundOffset = wraparoundOffset - 360.0;
    } else if (goalAng-prevGoalAng < -240.0) {  // 180 -> -180 wraparound 
      wraparoundOffset = wraparoundOffset + 360.0;
    }
    prevGoalAng = goalAng;

    turnMotor.set(ControlMode.MotionMagic, (wraparoundOffset+goalAng)*falconEncoderRes*turnGearRatio/360.0);
    driveMotor.set(ControlMode.Velocity, goalVel*falconEncoderRes*driveGearRatio/(10.0*wheelCirc));
  }

  public final SwerveModuleState getState() {
    return new SwerveModuleState(getVel(), Rotation2d.fromDegrees(getAngle()));
  }

  public final SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPos(), Rotation2d.fromDegrees(getAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  public final double getVel() {
    return driveMotor.getSelectedSensorVelocity(0)*10.0*wheelCirc/(falconEncoderRes*driveGearRatio);
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public final double getPos() {
    return driveMotor.getSelectedSensorPosition(0)*wheelCirc/(falconEncoderRes*driveGearRatio);
  }
  
  // Returns the angle of the wheel in degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). 
  public final double getAngle() {
    return turnMotor.getSelectedSensorPosition(0)*360.0/(falconEncoderRes*turnGearRatio);
  }
}