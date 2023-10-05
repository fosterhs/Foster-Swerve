package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Drivetrain {
  public static final double maxVel = 4; // User defined maximum speed of the robot. Unit: meters per second
  public static final double maxAngularVel = 2*Math.PI; // User defined maximum rotational speed of the robot. Unit: raidans per second
 
  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left.
  private static final Translation2d frontLeftPos = new Translation2d(0.225, 0.225);
  private static final Translation2d frontRightPos = new Translation2d(0.225, -0.225); 
  private static final Translation2d backRightPos = new Translation2d(-0.225, -0.225);
  private static final Translation2d backLeftPos = new Translation2d(-0.225, 0.225);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftPos, frontRightPos, backRightPos, backLeftPos);

  private final SwerveModule frontLeftModule = new SwerveModule(1, 2, 0, false); 
  private final SwerveModule frontRightModule = new SwerveModule(3, 4, 1, true);
  private final SwerveModule backRightModule = new SwerveModule(5, 6, 2, true);
  private final SwerveModule backLeftModule = new SwerveModule(7, 8, 3, false);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
  
  private final AHRS gyro = new AHRS();
  
  // Path Following
  private PathPlannerTrajectory path;
  private final Timer timer = new Timer();
  private final double pathXTol = 0.03;
  private final double pathYTol = 0.03;
  private final double pathAngTol = 3.0;
  private double maxPathVel = 0.8;
  private double maxPathAcc = 0.4;
  private boolean pathReversal = false;
 
  // Autonomous Swerve Controller Parameters
  private final double kP_move = 1;
  private final double kI_move = 0;
  private final double kD_move = 0;
  private final double I_moveMax = 4;
  private final double kP_turn = 1;
  private final double kI_turn = 0;
  private final double kD_turn = 0;
  private final double I_turnMax = 4;
  private final PIDController xController = new PIDController(kP_move, kI_move, kD_move);
  private final PIDController yController = new PIDController(kP_move, kI_move, kD_move);
  private final ProfiledPIDController turnController = new ProfiledPIDController(kP_turn, kI_turn, kD_turn, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel, Drivetrain.maxAngularVel));
  private final HolonomicDriveController swerveController = new HolonomicDriveController(xController, yController, turnController); // Defining the PID controllers and their constants for trajectory tracking.


  public Drivetrain() {
    xController.setIntegratorRange(-I_moveMax, I_moveMax);
    yController.setIntegratorRange(-I_moveMax, I_moveMax);
    turnController.setIntegratorRange(-I_turnMax, I_turnMax);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    gyro.calibrate();
    Timer.delay(2);
    gyro.zeroYaw();
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
  public void drive(double xVel, double yVel, double angVel, boolean fieldRelative) {
    SwerveModuleState[] moduleStates;
    if (fieldRelative) {
      moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, angVel, Rotation2d.fromDegrees(getAngPos())));
    } else {
      moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xVel, yVel, angVel));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVel);
    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backRightModule.setState(moduleStates[2]);
    backLeftModule.setState(moduleStates[3]);
    odometry.update(Rotation2d.fromDegrees(getAngPos()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
    updateDash();
    SmartDashboard.putNumber("xVel", xVel);
    SmartDashboard.putNumber("yVel", yVel);
    SmartDashboard.putNumber("angVel", angVel);
  }
  
  // Returns the angular position of the robot in degrees. The angular position is referenced to the starting angle of the robot. CCW is positive.
  public double getAngPos() {
    return -gyro.getYaw();
  }
  
  // Returns the odometry calculated x position of the robot in meters.
  public double getXPos() {
    return odometry.getPoseMeters().getX();
  }

  // Returns the odometry calculated y position of the robot in meters.
  public double getYPos() {
    return odometry.getPoseMeters().getY();
  }
  
  // Loads the path. All paths should be loaded during robotInit() since this call is computationally expensive.
  public void loadPath(String pathName, boolean resetOdometry) {
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal); // Uploading the PathPlanner trajectory to the program. The maximum acceleration and velocity can be set to suitable values for auto.
  }

  // Should be called immediately before the user would like the robot to begin tracking the path. If resetOdometry == true, this call sets the field position of the robot to the starting point of the path.
  public void resetPathController(boolean resetOdometry) {
    xController.reset();
    yController.reset();
    turnController.reset(getAngPos());
    timer.restart();
    if (resetOdometry) {
      PathPlannerState startingState = path.getInitialState();
      odometry.resetPosition(Rotation2d.fromDegrees(getAngPos()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()}, startingState.poseMeters);
    }
  }
  
  // Tracks the path. Should be called each period until the endpoint is reached.
  public void followPath() {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    ChassisSpeeds adjustedSpeeds = swerveController.calculate(new Pose2d(getXPos(), getYPos(), Rotation2d.fromDegrees(getAngPos())), currentGoal, currentGoal.holonomicRotation); // Calculates the required robot velocities to accurately track the trajectory.
    drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true); // Sets the robot to the correct velocities. 
    SmartDashboard.putNumber("pathXPos", currentGoal.poseMeters.getX());
    SmartDashboard.putNumber("pathYPos", currentGoal.poseMeters.getY());
    SmartDashboard.putNumber("pathAngPos", currentGoal.poseMeters.getRotation().getDegrees());
  }
  
  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  public boolean atEndpoint() {
    PathPlannerState endState = path.getEndState();
    return Math.abs(getAngPos() - endState.poseMeters.getRotation().getDegrees()) < pathAngTol 
    && Math.abs(getXPos() - endState.poseMeters.getX()) < pathXTol 
    && Math.abs(getYPos() - endState.poseMeters.getY()) < pathYTol;
  }
  
  // Path following parameters should be adjusted using these functions prior to calling loadPath().
  public void setMaxPathVel(double desiredMaxVel) {
    maxPathVel = desiredMaxVel;
  }

  public void setPathReversal(boolean desiredReversal) {
    pathReversal = desiredReversal;
  }

  public void setMaxPathAcc(double desiredMaxAcc) {
    maxPathAcc = desiredMaxAcc;
  }

  private void updateDash() {
    SmartDashboard.putNumber("xPos", getXPos());
    SmartDashboard.putNumber("yPos", getYPos());
    SmartDashboard.putNumber("angPos", getAngPos());
    SmartDashboard.putNumber("FL angle", frontLeftModule.getAngle());
    SmartDashboard.putNumber("FL pos", frontLeftModule.getPos());
    SmartDashboard.putNumber("FL vel", frontLeftModule.getVel());
    SmartDashboard.putNumber("FR angle", frontRightModule.getAngle());
    SmartDashboard.putNumber("FR pos", frontRightModule.getPos());
    SmartDashboard.putNumber("FR vel", frontRightModule.getVel());
    SmartDashboard.putNumber("BL angle", backLeftModule.getAngle());
    SmartDashboard.putNumber("BL pos", backLeftModule.getPos());
    SmartDashboard.putNumber("BL vel", backLeftModule.getVel());
    SmartDashboard.putNumber("BR angle", backRightModule.getAngle());
    SmartDashboard.putNumber("BR pos", backRightModule.getPos());
    SmartDashboard.putNumber("BR vel", backRightModule.getVel());
  }
}