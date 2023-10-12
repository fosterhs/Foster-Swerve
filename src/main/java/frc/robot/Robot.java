package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0); // Initializes the joystick.
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules and gyro)

  // Limits the acceleration of controller inputs. 
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAcc/Drivetrain.maxAngularVel);

  private final double minSpeedScaleFactor = 0.05; // The maximum speed of the robot when the throttle is at its minimum position, as a percentage of maxVel and maxAngularVel

  public void autonomousInit() {
    swerve.resetPathController(true); // Must be called immediately prior to following a Path Planner path using followPath().
  }

  public void autonomousPeriodic() {
    if (!swerve.atEndpoint()) {
      swerve.followPath(); // Follows the path that was previously loaded from Path Planner using loadPath().
    } else {
      swerve.drive(0,0,0,false);
    }
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    double speedScaleFactor = (-stick.getThrottle() + 1 + 2 * minSpeedScaleFactor) / (2 + 2 * minSpeedScaleFactor); // Creates a scale factor for the maximum speed of the robot based on the throttle position.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xSpeed = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double ySpeed = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double rotSpeed = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(),0.1))*Drivetrain.maxAngularVel*speedScaleFactor;

    swerve.drive(xSpeed, ySpeed, rotSpeed, true); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
  }

  public void robotInit() {
    swerve.loadPath("Test Path", true); // Loads the path. Can be called anytime prior to following the path. resetOdometry causes the robot's position to be reset to the starting point of the path.
    
    // Helps prevent loop overruns when the robot is first enabled. These calls cause the robot to initialize code in other parts of the program so it does not need to be initialized during autonomousInit() or teleopInit(), saving computational resources.
    swerve.resetPathController(true);
    swerve.followPath();
    swerve.atEndpoint();
    swerve.drive(0.1, 0, 0, false);
    swerve.resetOdometry();
    swerve.updateDash();
  }

  public void robotPeriodic() {
    swerve.updateDash();

    // Allows the driver to toggle whether each of the swerve modules is on. Useful in the case of an engine failure in match. 
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

    // Re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
    if (stick.getRawButtonPressed(11)) {
      swerve.resetGyro();
    }

    // Toggles the gyro on/off. Useful in the case of a gyro failure. A disabled gyro leads to loss of auto and field-oriented control. 
    if (stick.getRawButtonPressed(12)) {
      swerve.toggleGyro();
    }
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}
}