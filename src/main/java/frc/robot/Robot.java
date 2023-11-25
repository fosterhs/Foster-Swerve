package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0); // Initializes the joystick.
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules, gyro, and path follower)

  // Limits the acceleration of controller inputs. 
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAcc/Drivetrain.maxAngularVel);
  
  private final double minSpeedScaleFactor = 0.05; // The maximum speed of the robot when the throttle is at its minimum position, as a percentage of maxVel and maxAngularVel
  
  ProfiledPIDController angController = new ProfiledPIDController(0.14, 0.0, 0.0, new TrapezoidProfile.Constraints(1/4*Math.PI, 1/2*Math.PI));
  
  public void autonomousInit() {}

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.addVisionEstimate(); // Uses the limelight to estimate the position of the robot. Should only be called if limelight estimations are trustworthy (>1 april tag in sight, close to april tag...)
    rotateToAprilTag();
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    double speedScaleFactor = (-stick.getThrottle() + 1 + 2 * minSpeedScaleFactor) / (2 + 2 * minSpeedScaleFactor); // Creates a scale factor for the maximum speed of the robot based on the throttle position.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(),0.1))*Drivetrain.maxAngularVel*speedScaleFactor;
    
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    // Allows the driver to rotate the robot about each corner. Defaults to a center of rotation at the center of the robot.
    if (stick.getRawButton(7)) { // Front Left
      swerve.drive(xVel, yVel, angVel, true, 0.29, 0.29);
    } else if (stick.getRawButton(8)) { // Front Right
      swerve.drive(xVel, yVel, angVel, true, 0.29, -0.29);
    } else if (stick.getRawButton(9)) { // Back Left
      swerve.drive(xVel, yVel, angVel, true, -0.29, 0.29);
    } else if (stick.getRawButton(10)) { // Back Right
      swerve.drive(xVel, yVel, angVel, true, -0.29, -0.29);
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    }
  }

  public void robotInit() {
    swerve.loadPath("Test", 4.0, 2.0, false); // Loads the path. All paths should be loaded in robotInit() because this call is computationally expensive.
    // Helps prevent loop overruns when the robot is first enabled. These calls cause the robot to initialize code in other parts of the program so it does not need to be initialized during autonomousInit() or teleopInit(), saving computational resources.
    swerve.resetPathController();
    swerve.followPath(0);
    swerve.atEndpoint(0, 0.01, 0.01, 0.5);
    swerve.drive(0.1, 0.0, 0.0, false, 0.0, 0.0);
    swerve.resetOdometry(0, 0, 0);
    swerve.updateDash();
  }

  public void robotPeriodic() {
    swerve.updateDash();

    // Allows the driver to toggle whether each of the swerve modules is on. Useful in the case of an engine failure in match. 
    if (stick.getRawButtonPressed(5)) {
      swerve.toggleFL();
    }
    if (stick.getRawButtonPressed(6)) {
      swerve.toggleFR();
    }
    if (stick.getRawButtonPressed(3)) {
      swerve.toggleBL();
    }
    if (stick.getRawButtonPressed(4)) {
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

    // Toggles whether vision information is used to drive the robot.
    if (stick.getRawButtonPressed(2)); {
      swerve.toggleVision();
    }
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void rotateToAprilTag() {
    double tx = LimelightHelpers.getTX("");
    boolean tv = LimelightHelpers.getTV("");
    double output = angController.calculate(tx);
    if (!tv){
      swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
    } else {
      swerve.drive(0.0, 0.0, output, true, 0.0, 0.0);
    }
  }
}