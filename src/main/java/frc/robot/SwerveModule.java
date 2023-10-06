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
    double goalAngleFor = desiredState.angle.getDegrees();
    double goalAngleRev = goalAngleFor > 0.0 ? goalAngleFor - 180.0 : goalAngleFor + 180.0; // Instead of rotating to the input angle, the swerve module can rotate to a position 180 degrees off and reverse the input velocity to achieve the same result.
    double currAngle = getAngle();
    double currAngleMod360 = currAngle - Math.round(currAngle/360.0)*360.0; // Limits currAngle between -180 and 180 degrees. 
    double[] AngleDistsFor = calcAngleDists(currAngleMod360, goalAngleFor);
    double[] AngleDistsRev = calcAngleDists(currAngleMod360, goalAngleRev);
    double[] AngleDists = {AngleDistsFor[0], AngleDistsFor[1], AngleDistsRev[0], AngleDistsRev[1]};

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
    double outputAngle = 0.0;
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
  
  // Calcuates the angular distance between two angles on a circle. The first distance returned does not cross the 180/-180 discontinuity, while the second distance does cross this discontinuity.
  private double[] calcAngleDists(double angle1, double angle2) {
    double largerAngle;
    double smallerAngle;
    if (angle2 > angle1) {
      largerAngle = angle2;
      smallerAngle = angle1;
    } else {
      largerAngle = angle1;
      smallerAngle = angle2;
    }
    double[] distances = {largerAngle - smallerAngle, 360.0 - largerAngle + smallerAngle};
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

  public double getWheelEncoder() {
    return wheelEncoder.getAbsolutePosition()*360;
  }
}