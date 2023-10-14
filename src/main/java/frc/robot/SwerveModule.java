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
  private static final double currentLimit = 40.0; // Single motor current limit in amps.

  private final AnalogEncoder wheelEncoder;
  private final double wheelEncoderZero;
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  private final boolean invertDrive;

  // Motor error code tracking variables.
  private final int maxMotorErrors = 20; // The most times a configuration command can be unsuccesfully sent to a motor before a failure is declared and the motor is disabled. 
  private boolean driveMotorFailure = false; // Whether the drive motor has failed to configure correctly.
  private boolean turnMotorFailure = false; // Whether the turn motor has failed to configure correctly.
  private boolean moduleFailure = false; // Whether either the drive motor or the turn motor has failed to configure correctly.
  private boolean moduleDisabled = false; // Whether the module has been disabled by the driver.

  public SwerveModule(int turnID, int driveID, int encoderID, boolean _invertDrive, double _wheelEncoderZero) {
    wheelEncoderZero = _wheelEncoderZero;
    invertDrive = _invertDrive;
    wheelEncoder = new AnalogEncoder(encoderID);
    driveMotor = new WPI_TalonFX(driveID);
    turnMotor = new WPI_TalonFX(turnID);

    configDriveMotor();
    configTurnMotor();
    moduleDisabled = driveMotorFailure && turnMotorFailure;
    moduleFailure = turnMotorFailure || driveMotorFailure;
  }

  // Sets the swerve module to the given state (velocity and angle).
  public void setSMS(SwerveModuleState desiredState) {
    double goalAngleFor = desiredState.angle.getDegrees();
    double goalAngleRev = goalAngleFor > 0.0 ? goalAngleFor - 180.0 : goalAngleFor + 180.0; // Instead of rotating to the input angle, the swerve module can rotate to a position 180 degrees off and reverse the input velocity to achieve the same result.
    double currAngle = getAngle();
    double currAngleMod360 = currAngle - Math.round(currAngle/360.0)*360.0; // Limits currAngle between -180 and 180 degrees. 
    double[] AngleDists = {Math.abs(currAngleMod360 - goalAngleFor), 360.0 - Math.abs(currAngleMod360 - goalAngleFor)
      , Math.abs(currAngleMod360 - goalAngleRev), 360.0 - Math.abs(currAngleMod360 - goalAngleRev)}; // Calculates the 4 possible angluar distances to the forwards and reverse goals from the current angle.

    // Finds the minimum angular distance of the 4 options available. 
    int minIndex = 0;
    double minDist = AngleDists[0];
    for (int currIndex = 1; currIndex < AngleDists.length; currIndex++) {
      if (AngleDists[currIndex] < minDist) {
        minDist = AngleDists[currIndex];
        minIndex = currIndex;
      }
    }

    // Sets the output angle based on the minimum angular distance. Also reverses the velocity of the swerve module if the minimum distance is based on a reversed angle. 
    double outputAngle = currAngle;
    boolean reverseVel = false;
    if (minIndex == 0) { // Forward angle, does not cross 180/-180.
      outputAngle = goalAngleFor > currAngleMod360 ? currAngle + minDist : currAngle - minDist;
    } else if (minIndex == 1) { // Forward angle, crosses 180/-180.
      outputAngle = goalAngleFor > currAngleMod360 ? currAngle - minDist : currAngle + minDist;
    } else if (minIndex == 2) { // Reverse angle, does not cross 180/-180
      outputAngle = goalAngleRev > currAngleMod360 ? currAngle + minDist : currAngle - minDist;
      reverseVel = true;
    } else { // Reverse angle, crosses 180/-180
      outputAngle = goalAngleRev > currAngleMod360 ? currAngle - minDist : currAngle + minDist;
      reverseVel = true;
    }
    double goalVel = reverseVel ? -desiredState.speedMetersPerSecond : desiredState.speedMetersPerSecond;

    setAngle(outputAngle);
    setVel(goalVel);
  }
  
  // Returns the velocity and angle of the module.
  public SwerveModuleState getSMS() {
    return new SwerveModuleState(getVel(), Rotation2d.fromDegrees(getAngle()));
  }
  
  // Returns the postion and angle of the module.
  public SwerveModulePosition getSMP() {
    return new SwerveModulePosition(getPos(), Rotation2d.fromDegrees(getAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  public double getVel() {
    if (!driveMotorFailure && !moduleDisabled) {
      return driveMotor.getSelectedSensorVelocity(0)*10.0*wheelCirc/(falconEncoderRes*driveGearRatio);
    } else {
      return 0;
    }
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getPos() {
    if (!driveMotorFailure && !moduleDisabled) {
      return driveMotor.getSelectedSensorPosition(0)*wheelCirc/(falconEncoderRes*driveGearRatio);
    } else {
      return 0;
    }
  }
  
  // Returns the angle of the wheel in degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). 
  public double getAngle() {
    if (!turnMotorFailure && !moduleDisabled) {
      return turnMotor.getSelectedSensorPosition(0)*360.0/(falconEncoderRes*turnGearRatio);
    } else {
      return 0;
    }
  }
  
  // Returns the raw value of the wheel encoder. Range: 0-360 degrees.
  public double getWheelEncoder() {
    double wheelAngle = wheelEncoder.getAbsolutePosition()*360.0 - wheelEncoderZero;
    if (wheelAngle > 180.0) {
      wheelAngle = wheelAngle - 360.0;
    } else if (wheelAngle < -180.0) {
      wheelAngle = wheelAngle + 360.0;
    }
    return wheelAngle;
  }
  
  // Sets the velocity of the module. Units: meters per second
  private void setVel(double vel) {
    if (!driveMotorFailure && !moduleDisabled) {
      driveMotor.set(ControlMode.Velocity, vel*falconEncoderRes*driveGearRatio/(10.0*wheelCirc));
    } else {
      driveMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  
  // Sets the angle of the module. Units: degrees
  private void setAngle(double angle) {
    if (!turnMotorFailure && !moduleDisabled) {
      turnMotor.set(ControlMode.MotionMagic, angle*falconEncoderRes*turnGearRatio/360.0);
    } else {
      turnMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  // Toggles whether the module is enabled or disabled. Used in the case of an engine failure.
  public void toggleModule() {
    if (moduleDisabled) {
      enableDriveMotor();
      enableTurnMotor();
    } else {
      disableDriveMotor();
      disableTurnMotor();
    }
    moduleDisabled = !moduleDisabled;
    moduleFailure = turnMotorFailure || driveMotorFailure;
  }

  // True if the drive motor failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getDriveMotorFailure() {
    return driveMotorFailure;
  }

  // True if the turn motor failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getTurnMotorFailure() {
    return turnMotorFailure;
  }

  // True if the drive motor *or* turn motor failed to configure.
  public boolean getModuleFailure() {
    return moduleFailure;
  }

  // True if the drive motor *and* the turn motor failed to configure on startup, or if the driver disabled the module.
  public boolean getModuleDisabled() {
    return moduleDisabled;
  }

  // The following 2 functions disable the motor in the case of too many CAN errors, or if the driver chooses to disable the module in the case of an engine or mechanical failure.
  private void disableTurnMotor() {
    turnMotor.setNeutralMode(NeutralMode.Coast);
    turnMotor.set(ControlMode.PercentOutput, 0);
  }

  private void disableDriveMotor() {
    driveMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.set(ControlMode.PercentOutput, 0);
  }

  // The following 2 functions re-enable the motor. These should be called if the motors were previously disabled by the driver, or failed to properly initialize at robot start-up. Motors will automatically be re-disabled if they are not able to be configured properly.
  private void enableTurnMotor() {
    turnMotorFailure = false;
    configTurnMotor();
  }

  private void enableDriveMotor() {
    driveMotorFailure = false;
    configDriveMotor();
  }

  private void configDriveMotor() {
    driveMotor.setInverted(invertDrive);
    TalonFXConfiguration config = configCurrentLimit();

    // Sets the PID controller parameters
    double kI_drive = 0.0003;
    config.slot0.kP = 0.04;
    config.slot0.kI = kI_drive;
    config.slot0.kD = 1.0;
    config.slot0.kF = 0.0447;
    config.slot0.maxIntegralAccumulator = 0.8*1023.0/kI_drive;

    int driveMotorErrors = 0;

    while (driveMotor.configAllSettings(config, 30).value != 0) {
      driveMotorErrors++;
      driveMotorFailure = driveMotorErrors > maxMotorErrors;
      if (driveMotorFailure) {
        break;
      }
    }
    while (driveMotor.setSelectedSensorPosition(0, 0, 30).value != 0) {
      driveMotorErrors++;
      driveMotorFailure = driveMotorErrors > maxMotorErrors;
      if (driveMotorFailure) {
        break;
      }
    }
    driveMotor.setNeutralMode(NeutralMode.Brake);

    if (driveMotorFailure) {
      disableDriveMotor();
    }
  }

  private void configTurnMotor() {
    turnMotor.setInverted(true);
    TalonFXConfiguration config = configCurrentLimit();

    // Sets the PID controller parameters
    double kI_turn = 0.001;
    config.slot0.kP = 0.4;
    config.slot0.kI = kI_turn;
    config.slot0.kD = 3.0;
    config.slot0.maxIntegralAccumulator = 0.8*1023.0/kI_turn;
    config.slot0.allowableClosedloopError = 20;
    config.motionAcceleration = 100000;
    config.motionCruiseVelocity = 200000;

    int turnMotorErrors = 0;
    while (turnMotor.configAllSettings(config, 30).value != 0) {
      turnMotorErrors++;
      turnMotorFailure = turnMotorErrors > maxMotorErrors;
      if (turnMotorFailure) {
        break;
      }
    }
    while (turnMotor.setSelectedSensorPosition(getWheelEncoder()*falconEncoderRes*turnGearRatio/360.0).value != 0) {
      turnMotorErrors++;
      turnMotorFailure = turnMotorErrors > maxMotorErrors;
      if (turnMotorFailure) {
        break;
      }
    }
    turnMotor.setNeutralMode(NeutralMode.Brake);

    if (turnMotorFailure) {
      disableTurnMotor();
    }
  }

  // Creates a configuration object that limits the current draw for each motor to 40 amps. Used to configure both motors.
  private TalonFXConfiguration configCurrentLimit() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = currentLimit;
    config.supplyCurrLimit.triggerThresholdTime = 0.5;
    config.supplyCurrLimit.currentLimit = currentLimit;
    return config;
  }
}