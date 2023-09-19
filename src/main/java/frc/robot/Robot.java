package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick c = new Joystick(0);
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules, gyros, encoders)

  // Limits the acceleration of controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotAccLimiter = new SlewRateLimiter(3);
  
  Timer timer = new Timer();
  Trajectory trajectory;
  HolonomicDriveController autoController;

  @Override
  public void robotInit() {
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // State of the robot at the begining of the trajectory.
    Pose2d endPose = new Pose2d(1,1, Rotation2d.fromDegrees(0)); // state of the robot at the end of the trajectory.
    ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>(); // Any waypoints between the start and the end of the trajectory can be added here.
    waypoints.add(new Translation2d(1, 0));
    TrajectoryConfig config = new TrajectoryConfig(0.5, 0.2); // Setting the maximum velocity and acceleration of the robot during the trajectory.
    config.setKinematics(swerve.kin);
    trajectory = TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);
    autoController = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel/2, Drivetrain.maxAngularVel/4))); // Defining the PID controllers and their constants for trajectory tracking.
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    Trajectory.State goal = trajectory.sample(timer.get()); // Finds the desired state of the robot at the given time based on the trajectory.
    ChassisSpeeds adjustedSpeeds = autoController.calculate(new Pose2d(swerve.xPos, swerve.yPos, Rotation2d.fromDegrees(swerve.angPos)), goal, Rotation2d.fromDegrees(timer.get()*10)); // Calculates the required robot velocities to accurately track the trajectory.
    swerve.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true); // Sets the robot to the correct velocities. 
    swerve.updateOdometry();
    updateDash();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xSpeed = xAccLimiter.calculate(MathUtil.applyDeadband(-c.getY(),0.1))*Drivetrain.maxVel;
    double ySpeed = yAccLimiter.calculate(MathUtil.applyDeadband(-c.getX(),0.1))*Drivetrain.maxVel;
    double rotSpeed = rotAccLimiter.calculate(MathUtil.applyDeadband(-c.getZ(),0.1))*Drivetrain.maxAngularVel;

    swerve.drive(xSpeed, ySpeed, rotSpeed, true); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
    swerve.updateOdometry(); // Should be called every TimedRobot loop. Keeps track of the x-position, y-position, and angular position of the robot.
    updateDash();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void updateDash() {
    SmartDashboard.putNumber("xVel", swerve.xVel);
    SmartDashboard.putNumber("yVel", swerve.yVel);
    SmartDashboard.putNumber("angVel", swerve.angVel);
    SmartDashboard.putNumber("xPos", swerve.xPos);
    SmartDashboard.putNumber("yPos", swerve.yPos);
    SmartDashboard.putNumber("angPos", swerve.angPos);
    SmartDashboard.putNumber("FL angle", swerve.frontLeftModule.getAngle());
    SmartDashboard.putNumber("FL pos", swerve.frontLeftModule.getPos());
    SmartDashboard.putNumber("FL vel", swerve.frontLeftModule.getVel());
    SmartDashboard.putNumber("FR angle", swerve.frontRightModule.getAngle());
    SmartDashboard.putNumber("FR pos", swerve.frontRightModule.getPos());
    SmartDashboard.putNumber("FR vel", swerve.frontRightModule.getVel());
    SmartDashboard.putNumber("BL angle", swerve.backLeftModule.getAngle());
    SmartDashboard.putNumber("BL pos", swerve.backLeftModule.getPos());
    SmartDashboard.putNumber("BL vel", swerve.backLeftModule.getVel());
    SmartDashboard.putNumber("BR angle", swerve.backRightModule.getAngle());
    SmartDashboard.putNumber("BR pos", swerve.backRightModule.getPos());
    SmartDashboard.putNumber("BR vel", swerve.backRightModule.getVel());
  }
}