package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Drivetrain {
  public static final double maxVel = 4.0; // User defined maximum speed of the robot. Unit: meters per second
  public static final double maxAngularVel = Math.PI; // User defined maximum rotational speed of the robot. Unit: raidans per second
  
  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left.
  private final Translation2d frontLeftPos = new Translation2d(0.225, 0.225);
  private final Translation2d frontRightPos = new Translation2d(0.225, -0.225); 
  private final Translation2d backRightPos = new Translation2d(-0.225, -0.225);
  private final Translation2d backLeftPos = new Translation2d(-0.225, 0.225);

  private final SwerveModule frontLeftModule = new SwerveModule(1, 2, 0, false); 
  private final SwerveModule frontRightModule = new SwerveModule(3, 4, 1, true);
  private final SwerveModule backRightModule = new SwerveModule(5, 6, 2, true);
  private final SwerveModule backLeftModule = new SwerveModule(7, 8, 3, false);

  private final AHRS gyro = new AHRS();

  private final SwerveDriveKinematics kin = new SwerveDriveKinematics(frontLeftPos, frontRightPos, backRightPos, backLeftPos);
  private final SwerveDriveOdometry odo = new SwerveDriveOdometry(kin, new Rotation2d(), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});

  public Drivetrain() {
    gyro.calibrate();
    Timer.delay(2);
    gyro.zeroYaw();
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
  public void drive(double xVel, double yVel, double angVel) {
    SwerveModuleState[] moduleStates = kin.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, angVel, new Rotation2d(-gyro.getYaw()*Math.PI/180)));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVel);
    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backRightModule.setState(moduleStates[2]);
    backLeftModule.setState(moduleStates[3]);
    SmartDashboard.putNumber("xVel", xVel);
    SmartDashboard.putNumber("yVel", yVel);
    SmartDashboard.putNumber("angVel", angVel);
  }
  
  // Should be called every TimedRobot loop. Keeps track of the x-position, y-position, and angular position of the robot.
  public void updateOdometry() {
    odo.update(new Rotation2d(-gyro.getYaw()*Math.PI/180), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
    Pose2d pose = odo.getPoseMeters();
    SmartDashboard.putNumber("xPos", pose.getX());
    SmartDashboard.putNumber("yPos", pose.getY());
    SmartDashboard.putNumber("angPos", pose.getRotation().getDegrees());
  }
}