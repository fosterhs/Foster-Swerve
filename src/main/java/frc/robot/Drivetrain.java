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

  public final SwerveModule frontLeftModule = new SwerveModule(1, 2, 0, false); 
  public final SwerveModule frontRightModule = new SwerveModule(3, 4, 1, true);
  public final SwerveModule backRightModule = new SwerveModule(5, 6, 2, true);
  public final SwerveModule backLeftModule = new SwerveModule(7, 8, 3, false);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
  
  private final AHRS gyro = new AHRS();
  
  // Path Following
  private PathPlannerTrajectory path;
  private final Timer timer = new Timer();
  private double xTol = 0.03;
  private double yTol = 0.03;
  private double angTol = 3.0;
  private double maxPathVel = 0.8;
  private double maxPathAcc = 0.4;
  private boolean pathReversal = false;
 
  // Autonomous Swerve Controller Parameters
  private HolonomicDriveController swerveController;
  private final double kP_move = 1;
  private final double kI_move = 0;
  private final double kD_move = 0;
  private final double I_moveMax = 4;
  private final double kP_turn = 1;
  private final double kI_turn = 0;
  private final double kD_turn = 0;
  private final double I_turnMax = 4;

  public Drivetrain() {
    gyro.calibrate();
    Timer.delay(2);
    gyro.zeroYaw();
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
  public final void drive(double xVel, double yVel, double angVel, boolean fieldRelative) {
    SwerveModuleState[] moduleStates;
    if (fieldRelative) {
      moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, angVel, Rotation2d.fromDegrees(getYaw())));
    } else {
      moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xVel, yVel, angVel));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVel);
    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backRightModule.setState(moduleStates[2]);
    backLeftModule.setState(moduleStates[3]);
    updateOdometry();
    SmartDashboard.putNumber("xVel", xVel);
    SmartDashboard.putNumber("yVel", yVel);
    SmartDashboard.putNumber("angVel", angVel);
  }

  // Keeps track of the x-position, y-position, and angular position of the robot.
  public final void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getYaw()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()});
  }
  
  public final double getYaw() {
    return -gyro.getYaw();
  }
  
  public final double getRobotX() {
    return odometry.getPoseMeters().getX();
  }

  public final double getRobotY() {
    return odometry.getPoseMeters().getY();
  }
  
  // Loads the path. Should be called immediately before the user would like the robot to begin tracking the path. Assumes the robot is starting at the field position idicated by the start point of the path.
  public final void loadPath(String pathName) {
    PIDController xController = new PIDController(kP_move, kI_move, kD_move);
    xController.setIntegratorRange(-I_moveMax, I_moveMax);
    PIDController yController = new PIDController(kP_move, kI_move, kD_move);
    yController.setIntegratorRange(-I_moveMax, I_moveMax);
    ProfiledPIDController turnController = new ProfiledPIDController(kP_turn, kI_turn, kD_turn, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel, Drivetrain.maxAngularVel));
    turnController.setIntegratorRange(-I_turnMax, I_turnMax);
    swerveController = new HolonomicDriveController(xController, yController, turnController); // Defining the PID controllers and their constants for trajectory tracking.
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal); // Uploading the PathPlanner trajectory to the program. The maximum acceleration and velocity can be set to suitable values for auto.
    PathPlannerState startingState = path.getInitialState();
    odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()}, startingState.poseMeters);
    timer.restart();
  }
  
  // Tracks the path. Should be called each period until the endpoint is reached.
  public final void followPath() {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    Rotation2d currentAngGoal = currentGoal.holonomicRotation;
    ChassisSpeeds adjustedSpeeds = swerveController.calculate(new Pose2d(getRobotX(), getRobotY(), Rotation2d.fromDegrees(getYaw())), currentGoal, currentAngGoal); // Calculates the required robot velocities to accurately track the trajectory.
    drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true); // Sets the robot to the correct velocities. 
  }
  
  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  public final boolean atEndpoint() {
    PathPlannerState endState = path.getEndState();
    return Math.abs(getYaw() - endState.poseMeters.getRotation().getDegrees()) < angTol 
    && Math.abs(getRobotX() - endState.poseMeters.getX()) < xTol 
    && Math.abs(getRobotY() - endState.poseMeters.getY()) < yTol;
  }
  
  // Path following parameters should be adjusted using these functions prior to calling loadPath().
  public final void setPathXTol(double desiredXTol) {
    xTol = desiredXTol;
  }

  public final void setPathYTol(double desiredYTol) {
    yTol = desiredYTol;
  }

  public final void setPathAngTol(double desiredAngTol) {
    angTol = desiredAngTol;
  }

  public final void setMaxPathVel(double desiredMaxVel) {
    maxPathVel = desiredMaxVel;
  }

  public final void setPathReversal(boolean desiredReversal) {
    pathReversal = desiredReversal;
  }

  public final void setMaxPathAcc(double desiredMaxAcc) {
    maxPathAcc = desiredMaxAcc;
  }
}