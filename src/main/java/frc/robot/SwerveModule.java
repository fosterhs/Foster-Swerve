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

  private final AnalogEncoder wheelEncoder;
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;

  public SwerveModule(int turnID, int driveID, int encoderID, boolean invertDrive) {
    wheelEncoder = new AnalogEncoder(encoderID);
    driveMotor = new WPI_TalonFX(driveID);
    turnMotor = new WPI_TalonFX(turnID);
    
    while (driveMotor.configFactoryDefault(30).value != 0);
    while (turnMotor.configFactoryDefault(30).value != 0);
    
    // Limits current draw for each motor to 40 amps
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = currentLimit;
    config.supplyCurrLimit.triggerThresholdTime = 0.5;
    config.supplyCurrLimit.currentLimit = currentLimit;

    while (turnMotor.configAllSettings(config, 30).value != 0);
    while (turnMotor.setSelectedSensorPosition(0, 0, 30).value != 0);
    turnMotor.setNeutralMode(NeutralMode.Brake);
    
    // Motion Magic parameters for the turning motor
    double kI_turn = 0.001;
    while (turnMotor.config_kP(0, 0.4, 30).value != 0);
    while (turnMotor.config_kI(0, kI_turn, 30).value != 0);
    while (turnMotor.config_kD(0, 3.0, 30).value != 0);
    while (turnMotor.configMotionAcceleration(100000, 30).value != 0);
    while (turnMotor.configMotionCruiseVelocity(200000, 30).value != 0);
    while (turnMotor.configAllowableClosedloopError(0, 20, 30).value != 0);
    while (turnMotor.configMaxIntegralAccumulator(0, 0.8*1023/kI_turn, 30).value != 0);

    while (driveMotor.configAllSettings(config, 30).value != 0);
    while (driveMotor.setSelectedSensorPosition(0, 0, 30).value != 0);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    
    // Velocity control parameters for the drive motor
    double kI_drive = 0.0003;
    while (driveMotor.config_kP(0, 0.04, 30).value != 0);
    while (driveMotor.config_kI(0, kI_drive, 30).value != 0);
    while (driveMotor.config_kD(0, 1.0, 30).value != 0);
    while (driveMotor.config_kF(0, 0.0447, 30).value != 0);
    while (driveMotor.configMaxIntegralAccumulator(0, 0.8*1023.0/kI_drive, 30).value != 0);

    if (invertDrive) {
      driveMotor.setInverted(true);
    }
    turnMotor.setInverted(true);

    System.out.println("Motors " + String.valueOf(turnID) + " and " + String.valueOf(encoderID) + " configured successfully.");
  }
  
  // Sets the swerve module to the given state.
  public void setState(SwerveModuleState desiredState) {
    double goalAngleFor = desiredState.angle.getDegrees();
    double goalAngleRev = goalAngleFor > 0.0 ? goalAngleFor - 180.0 : goalAngleFor + 180.0; // Instead of rotating to the input angle, the swerve module can rotate to a position 180 degrees off and reverse the input velocity to achieve the same result.
    double currAngle = getAngle();
    double currAngleMod360 = currAngle - Math.round(currAngle/360.0)*360.0; // Limits currAngle between -180 and 180 degrees. 
    double[] AngleDists = {Math.abs(currAngleMod360 - goalAngleFor), 360.0 - Math.abs(currAngleMod360 - goalAngleFor), Math.abs(currAngleMod360 - goalAngleRev), 360.0 - Math.abs(currAngleMod360 - goalAngleRev)}; // Calculates the 4 possible angluar distances to the forwards and reverse goals from the current angle.

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
    }
    if (minIndex == 1) { // Forward angle, crosses 180/-180.
      outputAngle = goalAngleFor > currAngleMod360 ? currAngle - minDist : currAngle + minDist;
    }
    if (minIndex == 2) { // Reverse angle, does not cross 180/-180
      outputAngle = goalAngleRev > currAngleMod360 ? currAngle + minDist : currAngle - minDist;
      reverseVel = true;
    }
    if (minIndex == 3) { // Reverse angle, crosses 180/-180
      outputAngle = goalAngleRev > currAngleMod360 ? currAngle - minDist : currAngle + minDist;
      reverseVel = true;
    }
    double goalVel = reverseVel ? -desiredState.speedMetersPerSecond : desiredState.speedMetersPerSecond;

    turnMotor.set(ControlMode.MotionMagic, outputAngle*falconEncoderRes*turnGearRatio/360.0);
    driveMotor.set(ControlMode.Velocity, goalVel*falconEncoderRes*driveGearRatio/(10.0*wheelCirc));
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

  public double getWheelEncoder() {
    return wheelEncoder.getAbsolutePosition()*360;
  }
}