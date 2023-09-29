package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0);
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules, gyros, encoders)

  // Limits the acceleration of controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(3);
  
  // Trajectory following objects
  Timer timer = new Timer();
  HolonomicDriveController autoCont = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel, Drivetrain.maxAngularVel))); // Defining the PID controllers and their constants for trajectory tracking.
  PathPlannerTrajectory path = PathPlanner.loadPath("Test Path", new PathConstraints(4, 4)); // Uploading the PathPlanner trajectory to the program. The maximum acceleration and velocity can be set to suitable values for auto.

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    Rotation2d currentAngleGoal = currentGoal.holonomicRotation;
    ChassisSpeeds adjustedSpeeds = autoCont.calculate(new Pose2d(swerve.xPos, swerve.yPos, Rotation2d.fromDegrees(swerve.angPos)), currentGoal, currentAngleGoal); // Calculates the required robot velocities to accurately track the trajectory.
    swerve.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true); // Sets the robot to the correct velocities. 
    swerve.updateOdometry();
    updateDash();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xSpeed = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(),0.1))*Drivetrain.maxVel;
    double ySpeed = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(),0.1))*Drivetrain.maxVel;
    double rotSpeed = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(),0.1))*Drivetrain.maxAngularVel;

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
    SmartDashboard.putNumber("FL desAngle", swerve.frontLeftModule.goalAng);
    SmartDashboard.putNumber("FR angle", swerve.frontRightModule.getAngle());
    SmartDashboard.putNumber("FR pos", swerve.frontRightModule.getPos());
    SmartDashboard.putNumber("FR vel", swerve.frontRightModule.getVel());
    SmartDashboard.putNumber("FR desAngle", swerve.frontRightModule.goalAng);
    SmartDashboard.putNumber("BL angle", swerve.backLeftModule.getAngle());
    SmartDashboard.putNumber("BL pos", swerve.backLeftModule.getPos());
    SmartDashboard.putNumber("BL vel", swerve.backLeftModule.getVel());
    SmartDashboard.putNumber("BL desAngle", swerve.backLeftModule.goalAng);
    SmartDashboard.putNumber("BR angle", swerve.backRightModule.getAngle());
    SmartDashboard.putNumber("BR pos", swerve.backRightModule.getPos());
    SmartDashboard.putNumber("BR vel", swerve.backRightModule.getVel());
    SmartDashboard.putNumber("BR desAngle", swerve.backRightModule.goalAng);
  }
}