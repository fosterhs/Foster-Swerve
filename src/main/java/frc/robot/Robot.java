package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0);
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules)

  // Limits the acceleration of controller inputs. A limit of 3 corresponds to 1/3 of a second to reach max output from 0.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(3.0);

  public void autonomousInit() {
    swerve.resetPathController(true);
  }

  public void autonomousPeriodic() {
    swerve.followPath(); 
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xSpeed = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(),0.1))*Drivetrain.maxVel;
    double ySpeed = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(),0.1))*Drivetrain.maxVel;
    double rotSpeed = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(),0.1))*Drivetrain.maxAngularVel;

    swerve.drive(xSpeed, ySpeed, rotSpeed, true); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
  }

  public void robotInit() {
    swerve.loadPath("Test Path", true); // Loads the path. Should be called prior to following the path. resetOdometry causes the robot's position to be set to the starting point of the path.
    swerve.drive(0, 0, 0, true); // Helps prevent loop overruns when the robot is first enabled. This call causes the robot to initialize code in other parts of the program, so it does not need to be initialized during autonomousInit() or teleopInit().
  }

  public void robotPeriodic() {}

  public void disabledInit() {}

  public void disabledPeriodic() {}
};