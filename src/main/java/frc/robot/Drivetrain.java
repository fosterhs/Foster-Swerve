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

class Drivetrain {
  public static final double maxVel = 4; // User defined maximum speed of the robot. Unit: meters per second
  public static final double maxAngularVel = Math.PI; // User defined maximum rotational speed of the robot. Unit: raidans per second
  
  // Odometry variables
  public double xVel = 0;
  public double yVel = 0;
  public double angVel = 0;
  public double xPos = 0;
  public double yPos = 0;
  public double angPos = 0;
  
  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left.
  private Translation2d frontLeftPos = new Translation2d(0.225, 0.225);
  private Translation2d frontRightPos = new Translation2d(0.225, -0.225); 
  private Translation2d backRightPos = new Translation2d(-0.225, -0.225);
  private Translation2d backLeftPos = new Translation2d(-0.225, 0.225);

  public SwerveModule frontLeftModule = new SwerveModule(1, 2, 0, false); 
  public SwerveModule frontRightModule = new SwerveModule(3, 4, 1, true);
  public SwerveModule backRightModule = new SwerveModule(5, 6, 2, true);
  public SwerveModule backLeftModule = new SwerveModule(7, 8, 3, false);

  private AHRS gyro = new AHRS();
  
  private SwerveDriveKinematics kin = new SwerveDriveKinematics(frontLeftPos, frontRightPos, backRightPos, backLeftPos);
  private SwerveDriveOdometry odo = new SwerveDriveOdometry(kin, new Rotation2d(), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
  
  // Path Following
  private HolonomicDriveController autoCont;
  private PathPlannerTrajectory path;
  private Timer timer;
  private double xTol = 0.03;
  private double yTol = 0.03;
  private double angTol = 3.0;
  private double maxPathVel = 0.8;
  private double maxPathAcc = 0.4;
  private boolean pathReversal = false;

  public Drivetrain() {
    gyro.calibrate();
    Timer.delay(2);
    gyro.zeroYaw();
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
  public void drive(double xVelCommanded, double yVelCommanded, double angVelCommanded, boolean fieldRelative) {
    SwerveModuleState[] moduleStates;
    if (fieldRelative) {
      moduleStates = kin.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVelCommanded, yVelCommanded, angVelCommanded, Rotation2d.fromDegrees(-gyro.getYaw())));
    } else {
      moduleStates = kin.toSwerveModuleStates(new ChassisSpeeds(xVelCommanded, yVelCommanded, angVelCommanded));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVel);
    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backRightModule.setState(moduleStates[2]);
    backLeftModule.setState(moduleStates[3]);
    xVel = xVelCommanded;
    yVel = yVelCommanded;
    angVel = angVelCommanded;
    updateOdometry();
  }
  
  // Keeps track of the x-position, y-position, and angular position of the robot.
  public void updateOdometry() {
    odo.update(Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
    Pose2d robotPose = odo.getPoseMeters();
    xPos = robotPose.getX();
    yPos = robotPose.getY();
    angPos = robotPose.getRotation().getDegrees();
  }

  public void loadPath(String pathName) {
    autoCont = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel, Drivetrain.maxAngularVel))); // Defining the PID controllers and their constants for trajectory tracking.
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal); // Uploading the PathPlanner trajectory to the program. The maximum acceleration and velocity can be set to suitable values for auto.
    PathPlannerState startingState = path.getInitialState();
    odo = new SwerveDriveOdometry(kin, Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()}, startingState.poseMeters);
    timer = new Timer();
    timer.start();
  }

  public void followPath() {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    Rotation2d currentAngleGoal = currentGoal.holonomicRotation;
    ChassisSpeeds adjustedSpeeds = autoCont.calculate(new Pose2d(xPos, yPos, Rotation2d.fromDegrees(angPos)), currentGoal, currentAngleGoal); // Calculates the required robot velocities to accurately track the trajectory.
    drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true); // Sets the robot to the correct velocities. 
  }

  public boolean atEndpoint() {
    PathPlannerState endState = path.getEndState();
    return Math.abs(odo.getPoseMeters().getRotation().getDegrees() - endState.poseMeters.getRotation().getDegrees()) < angTol 
    && Math.abs(odo.getPoseMeters().getX() - endState.poseMeters.getX()) < xTol 
    && Math.abs(odo.getPoseMeters().getY() - endState.poseMeters.getY()) < yTol;
  }

  public void setPathXTol(double desiredXTol) {
    xTol = desiredXTol;
  }

  public void setPathYTol(double desiredYTol) {
    yTol = desiredYTol;
  }

  public void setPathAngTol(double desiredAngTol) {
    angTol = desiredAngTol;
  }

  public void setMaxPathVel(double desiredMaxVel) {
    maxPathVel = desiredMaxVel;
  }

  public void setPathReversal(boolean desiredReversal) {
    pathReversal = desiredReversal;
  }

  public void setMaxPathAcc(double desiredMaxAcc) {
    maxPathAcc = desiredMaxAcc;
  }
}