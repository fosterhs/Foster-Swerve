package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

class SwerveModule {
  private static final double wheelCirc = 2*0.0254*2*Math.PI; // Circumference of the wheel. Unit: meters
  private static final double falconEncoderRes = 2048;
  private static final double turnGearRatio = 150/7.1;
  private static final double driveGearRatio = 57/7;
  private static final double currentLimit = 40;
  private static final double wheelEncoderRes = 4096;

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  private final AnalogEncoder wheelEncoder;
  
  // Keeps track wraparounds when the wheel angle crosses 180 degrees 
  private double previousAngle = 0;
  private double wraparoundOffset = 0;

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
    turnMotor.config_kP(0, 0.4);
    turnMotor.config_kI(0, 0.001);
    turnMotor.config_kD(0, 3);
    turnMotor.configMotionAcceleration(100000);
    turnMotor.configMotionCruiseVelocity(200000);
    turnMotor.configAllowableClosedloopError(0, 50);

    driveMotor.configAllSettings(config);
    driveMotor.setSelectedSensorPosition(0);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    
    // Velocity control parameters for the drive motor
    driveMotor.config_kP(0, 0.04);
    driveMotor.config_kI(0, 0.0003);
    driveMotor.config_kD(0, 1);
    driveMotor.config_kF(0, 0.0447);

    if (invertDrive) {
      driveMotor.setInverted(true);
    }
  }
  
  // Sets the swerve module to the given state.
  public void setState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
    double desiredVel = optimizedState.speedMetersPerSecond;
    double desiredAngle = optimizedState.angle.getDegrees();
    
    // Handles wraparounds that occur at 180/-180 degrees
    if (desiredAngle-previousAngle > 300) {
      wraparoundOffset = wraparoundOffset + 360;
    } else if (desiredAngle-previousAngle < -300) {
      wraparoundOffset = wraparoundOffset - 360;
    }
    previousAngle = desiredAngle;

    turnMotor.set(ControlMode.MotionMagic, (wraparoundOffset-desiredAngle)*falconEncoderRes*turnGearRatio/360);
    driveMotor.set(ControlMode.Velocity, desiredVel*falconEncoderRes*driveGearRatio/(10*wheelCirc));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVel(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPos(), Rotation2d.fromDegrees(getAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  private double getVel() {
    return driveMotor.getSelectedSensorVelocity(0)*10*wheelCirc/(falconEncoderRes*driveGearRatio);
  }

  // Returns total distance the wheel has rotated. Unit: meters
  private double getPos() {
    return driveMotor.getSelectedSensorPosition(0)*wheelCirc/(falconEncoderRes*driveGearRatio);
  }
  
  // Returns the angle of the wheel in degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). 
  private double getAngle() {
    return -turnMotor.getSelectedSensorPosition(0)*360/(falconEncoderRes*turnGearRatio);
  }
}