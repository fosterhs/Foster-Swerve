package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0);
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules)

  // Limits the acceleration of controller inputs. 
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAcc/Drivetrain.maxAngularVel);
  private final double minSpeedScaleFactor = 0.05; // The speed of the robot when the throttle is at its minimum position, as a percentage of maxVel.

  public void autonomousInit() {
    swerve.resetPathController(true);
  }

  public void autonomousPeriodic() {
    swerve.followPath(); 
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double speedScaleFactor = (-stick.getThrottle() + 1 + 2 * minSpeedScaleFactor) / (2 + 2 * minSpeedScaleFactor);
    double xSpeed = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double ySpeed = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double rotSpeed = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(),0.1))*Drivetrain.maxAngularVel*speedScaleFactor;

    swerve.drive(xSpeed, ySpeed, rotSpeed, true); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
  }

  public void robotInit() {
    swerve.loadPath("Test Path", true); // Loads the path. Should be called prior to following the path. resetOdometry causes the robot's position to be set to the starting point of the path.
    
    // Helps prevent loop overruns when the robot is first enabled. This call causes the robot to initialize code in other parts of the program, so it does not need to be initialized during autonomousInit() or teleopInit().
    swerve.followPath();
    swerve.atEndpoint();
    swerve.resetPathController(true);
    swerve.resetOdometry();
    swerve.updateDash();
    swerve.drive(0.1, 0, 0, false);

  }

  public void robotPeriodic() {
    swerve.updateDash();

    if (stick.getRawButtonPressed(7)) {
      swerve.toggleFL();
    }
    if (stick.getRawButtonPressed(8)) {
      swerve.toggleFR();
    }
    if (stick.getRawButtonPressed(9)) {
      swerve.toggleBL();
    }
    if (stick.getRawButtonPressed(10)) {
      swerve.toggleBR();
    }
    if (stick.getRawButtonPressed(11)) {
      swerve.resetGyro();
    }
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}
}