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
  private double goalAng = 0.0;
  private double prevGoalAng = 0.0;
  public double wraparoundOffset = 0.0;

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
  public void setState(SwerveModuleState desiredState) {
    double goalVel = desiredState.speedMetersPerSecond;
    double goalAng = desiredState.angle.getDegrees();
    double currentAng = getAngle();
    double currentAngMod = getAngle() - Math.round(getAngle()/360)*360;
    double reverseGoalAng;
    if (goalAng > 0) {
      reverseGoalAng = goalAng - 180.0;
    } else {
      reverseGoalAng = goalAng + 180.0;
    }
    double[] distances = {calcDistances(currentAngMod, goalAng)[0], calcDistances(currentAngMod, goalAng)[1], calcDistances(currentAngMod, reverseGoalAng)[0], calcDistances(currentAngMod, reverseGoalAng)[1]};
    int minIndex = -1;
    double minDistance = 360;
    for (int i = 0; i < distances.length; i++) {
      if (distances[i] < minDistance) {
        minDistance = distances[i];
        minIndex = i;
      }
    }
    double output = 0;
    boolean reverse = false;
    if (minIndex == 0) {
      if (goalAng > currentAngMod) {
        output = currentAng + minDistance;
      } else {
        output = currentAng - minDistance;
      }
    }
    if (minIndex == 1) {
      if (goalAng > currentAngMod) {
        output = currentAng - minDistance;
      } else {
        output = currentAng + minDistance;
      }
    }
    if (minIndex == 2) {
      if (reverseGoalAng > currentAngMod) {
        output = currentAng + minDistance;
        reverse = true;
      } else {
        output = currentAng - minDistance;
        reverse = true;
      }
    }
    if (minIndex == 3) {
      if (reverseGoalAng > currentAngMod) {
        output = currentAng - minDistance;
        reverse = true;
      } else {
        output = currentAng + minDistance;
        reverse = true;
      }
    }
    
    if(reverse) {
      goalVel = -goalVel;
    }

    turnMotor.set(ControlMode.MotionMagic, output*falconEncoderRes*turnGearRatio/360.0);
    driveMotor.set(ControlMode.Velocity, goalVel*falconEncoderRes*driveGearRatio/(10.0*wheelCirc));
    
    /*
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
    double goalVel = optimizedState.speedMetersPerSecond;
    goalAng = optimizedState.angle.getDegrees();
    // Handles wraparounds that occur at 180/-180 degrees
    if (goalAng-prevGoalAng > 270.0) {  // -180 -> 180 wraparound 
      wraparoundOffset = wraparoundOffset - 360.0;
    } else if (goalAng-prevGoalAng < -270.0) {  // 180 -> -180 wraparound 
      wraparoundOffset = wraparoundOffset + 360.0;
    }
    prevGoalAng = goalAng;

    turnMotor.set(ControlMode.MotionMagic, (goalAng + wraparoundOffset)*falconEncoderRes*turnGearRatio/360.0);
    driveMotor.set(ControlMode.Velocity, goalVel*falconEncoderRes*driveGearRatio/(10.0*wheelCirc));
    */
  }

  public double[] calcDistances(double a, double b) {
    double larger;
    double smaller;
    if (b > a) {
      larger = b;
      smaller = a;
    } else {
      larger = a;
      smaller = b;
    }
    double[] distances = {larger - smaller, 360 - larger + smaller};
    return distances;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVel(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPos(), Rotation2d.fromDegrees(getAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  public double getVel() {
    return driveMotor.getSelectedSensorVelocity(0)*10.0*wheelCirc/(falconEncoderRes*driveGearRatio);
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getPos() {
    return driveMotor.getSelectedSensorPosition(0)*wheelCirc/(falconEncoderRes*driveGearRatio);
  }
  
  // Returns the angle of the wheel in degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). 
  public double getAngle() {
    return turnMotor.getSelectedSensorPosition(0)*360.0/(falconEncoderRes*turnGearRatio);
  }

  public double getGoalAngle() {
    return goalAng;
  }
}