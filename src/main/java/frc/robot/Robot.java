package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick c = new Joystick(0);
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules, gyros, encoders)

  // Limits the acceleration of controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotAccLimiter = new SlewRateLimiter(3);
  
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xSpeed = xAccLimiter.calculate(MathUtil.applyDeadband(-c.getY(),0.1))*Drivetrain.maxVel;
    double ySpeed = yAccLimiter.calculate(MathUtil.applyDeadband(-c.getX(),0.1))*Drivetrain.maxVel;
    double rotSpeed = rotAccLimiter.calculate(MathUtil.applyDeadband(-c.getZ(),0.1))*Drivetrain.maxAngularVel;

    swerve.drive(xSpeed, ySpeed, rotSpeed); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
    swerve.updateOdometry(); // Should be called every TimedRobot loop. Keeps track of the x-position, y-position, and angular position of the robot.

    SmartDashboard.putNumber("xVel", swerve.xVel);
    SmartDashboard.putNumber("yVel", swerve.yVel);
    SmartDashboard.putNumber("angVel", swerve.angVel);
    SmartDashboard.putNumber("xPos", swerve.xPos);
    SmartDashboard.putNumber("yPos", swerve.yPos);
    SmartDashboard.putNumber("angPos", swerve.angPos);
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
}