package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0);
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules, gyros, encoders)

  // Limits the acceleration of controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(3);

  public void autonomousInit() {
    swerve.loadPath("Test Path");
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

    swerve.drive(xSpeed, ySpeed, rotSpeed, true); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
  }

  public void robotInit() {
    
  }
  
  public void robotPeriodic() {
    updateDash();
  }

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