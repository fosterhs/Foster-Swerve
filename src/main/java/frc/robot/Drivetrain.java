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
  public static final double maxVel = 4.0; // User defined maximum speed of the robot. Unit: meters per second
  public static final double maxAngularVel = 2*Math.PI; // User defined maximum rotational speed of the robot. Unit: raidans per second
  public static final double maxAcc = 12.0; // User defined maximum acceleration of the robot. Unit: meters per second^2
  public static final double maxAngularAcc = 6*Math.PI; // User defined maximum rotational acceleration of the robot. Unit: raidans per second^2

  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left.
  private static final Translation2d frontLeftPos = new Translation2d(0.225, 0.225);
  private static final Translation2d frontRightPos = new Translation2d(0.225, -0.225); 
  private static final Translation2d backRightPos = new Translation2d(-0.225, -0.225);
  private static final Translation2d backLeftPos = new Translation2d(-0.225, 0.225);
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftPos, frontRightPos, backRightPos, backLeftPos);

  // Initializes each swerve module object.
  public final SwerveModule frontLeftModule = new SwerveModule(1, 2, 0, false, 113.4); 
  public final SwerveModule frontRightModule = new SwerveModule(3, 4, 1, true, 316.1);
  public final SwerveModule backRightModule = new SwerveModule(5, 6, 2, true, 74.2);
  public final SwerveModule backLeftModule = new SwerveModule(7, 8, 3, false, 141.1);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new SwerveModulePosition[] {frontLeftModule.getSMP(), frontRightModule.getSMP(), backRightModule.getSMP(), backLeftModule.getSMP()});
  public boolean moduleError; // Indicates whether there is at least 1 swerve module failure.
  public boolean moduleOffline; // Indcates whether at least 1 module is offline.

  // Gyroscope variables
  private final AHRS gyro = new AHRS();
  public boolean gyroFailure = false; // Indicates whether the gyro has lost connection at any point after a yaw-reset.
  public boolean gyroDisabled = false;
  
  // Path following parameters
  private final double pathXTol = 0.03; // Used to calcualte whether the robot is at the endpoint of a path. Hard code this value. Units: meters
  private final double pathYTol = 0.03; // Used to calcualte whether the robot is at the endpoint of a path. Hard code this value. Units: meters
  private final double pathAngTol = 3.0; // Used to calcualte whether the robot is at the endpoint of a path. Hard code this value. Units: degrees
  private double maxPathVel = 2.0; // Maximum robot velocity while following a path. Use the set function to modify this prior to following a path.
  private double maxPathAcc = 2.0; // Maximum robot acceleration while following a path. Use the set function to modify this prior to following a path.
  private boolean pathReversal = false; // Whether the path should be followed in forwards or reverse. Use the set function to modify this prior to following a path.

  // Path following
  private PathPlannerTrajectory path;
  private final Timer timer = new Timer();
 
  // Autonomous swerve controller parameters. Hard code these values.
  private final double kP_drive = 10;
  private final double kI_drive = 0;
  private final double kD_drive = 0;
  private final double I_driveMax = 0.6;
  private final double kP_turn = 5;
  private final double kI_turn = 0;
  private final double kD_turn = 0;
  private final double I_turnMax = 0.6;

  // Autonomous swerve controller
  private final PIDController xController = new PIDController(kP_drive, kI_drive, kD_drive);
  private final PIDController yController = new PIDController(kP_drive, kI_drive, kD_drive);
  private final ProfiledPIDController turnController = new ProfiledPIDController(kP_turn, kI_turn, kD_turn, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel, Drivetrain.maxAngularVel));
  private final HolonomicDriveController swerveController = new HolonomicDriveController(xController, yController, turnController); // Defining the PID controllers and their constants for trajectory tracking.
  
  // These variables are updated each period and passed to SmartDashboard.
  private double xVel = 0;
  private double yVel = 0;
  private double angVel = 0;
  private double pathXPos = 0;
  private double pathYPos = 0;
  private double pathAngPos = 0;

  public Drivetrain() {
    // Sets autonomous swerve controller parameters
    xController.setIntegratorRange(-I_driveMax, I_driveMax);
    yController.setIntegratorRange(-I_driveMax, I_driveMax);
    turnController.setIntegratorRange(-I_turnMax, I_turnMax);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    Timer.delay(2); // Delay to give the gyro time for start-up calibration.
    resetGyro(); // Sets the gyro angle to 0 based on the current heading of the robot.

    updateModuleStatus(); // Checks whether any modules are offline or did not start up properly.
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel
  public void drive(double _xVel, double _yVel, double _angVel, boolean fieldRelative) {
    xVel = _xVel;
    yVel = _yVel;
    angVel = _angVel;
    SwerveModuleState[] moduleStates;
    if (fieldRelative && !gyroFailure && !gyroDisabled) { // Field oriented control
      moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, angVel, Rotation2d.fromDegrees(getAngPos())));
    } else { // Robot oriented control
      moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xVel, yVel, angVel));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVel); // Makes sure the calculated velocities are attainable. If they are not, all modules velocities are scaled back.

    // Sets the module angles and velocities.
    frontLeftModule.setSMS(moduleStates[0]);
    frontRightModule.setSMS(moduleStates[1]);
    backRightModule.setSMS(moduleStates[2]);
    backLeftModule.setSMS(moduleStates[3]);
  }
  
  // Loads the path. All paths should be loaded during robotInit() since this call is computationally expensive.
  public void loadPath(String pathName, boolean resetOdometry) {
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal); // Uploading the PathPlanner trajectory to the program. The maximum acceleration and velocity can be set to suitable values for auto.
  }

  // Path following 3 functions should be adjusted prior to calling loadPath(). These functions define the parameters of the path follower. They will default to reasonable pre-defined values if these functions are not called.
  public void setMaxPathVel(double desiredMaxVel) {
    maxPathVel = desiredMaxVel;
  }

  public void setPathReversal(boolean desiredReversal) {
    pathReversal = desiredReversal;
  }

  public void setMaxPathAcc(double desiredMaxAcc) {
    maxPathAcc = desiredMaxAcc;
  }

  // Should be called immediately before the user would like the robot to begin tracking the path. If resetOdometry == true, this call sets the field position of the robot to the starting point of the path.
  public void resetPathController(boolean resetOdometryToPathStart) {
    xController.reset();
    yController.reset();
    turnController.reset(0);
    timer.restart();
    if (resetOdometryToPathStart) {
      resetOdometryToPathStart();
    }
  }
  
  // Tracks the path. Should be called each period until the endpoint is reached.
  public void followPath() {
    if (!gyroFailure && !gyroDisabled && !moduleError && !moduleOffline) {
      updateOdometry();
      PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
      ChassisSpeeds adjustedSpeeds = swerveController.calculate(new Pose2d(getXPos(), getYPos(), Rotation2d.fromDegrees(getAngPos())), currentGoal, currentGoal.holonomicRotation); // Calculates the required robot velocities to accurately track the trajectory.
      drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true); // Sets the robot to the correct velocities. 
      pathXPos = currentGoal.poseMeters.getX();
      pathYPos = currentGoal.poseMeters.getY();
      pathAngPos = currentGoal.holonomicRotation.getDegrees();
    } else {
      drive(0, 0, 0, false);
    }
  }
  
  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  public boolean atEndpoint() {
    if (!gyroDisabled && !gyroFailure && !moduleError && !moduleOffline) {
      PathPlannerState endState = path.getEndState();
      return Math.abs(getAngPos() - endState.holonomicRotation.getDegrees()) < pathAngTol 
      && Math.abs(getXPos() - endState.poseMeters.getX()) < pathXTol 
      && Math.abs(getYPos() - endState.poseMeters.getY()) < pathYTol;
    } else {
      return false;
    }
  }

  // Updates the position of the robot on the field. Should be called each period to remain accurate. Tends to noticably drift for periods of time >15 sec.
  public void updateOdometry() {
    if (!gyroDisabled && !gyroFailure) {
      odometry.update(Rotation2d.fromDegrees(getAngPos()), new SwerveModulePosition[] {frontLeftModule.getSMP(), frontRightModule.getSMP(), backRightModule.getSMP(), backLeftModule.getSMP()});
    }
  }
  
  // Returns the angular position of the robot in degrees. The angular position is referenced to the starting angle of the robot. CCW is positive. Will return 0 in the case of a gyro failure.
  public double getAngPos() {
    if (!gyroFailure && !gyroDisabled) {
      return -gyro.getYaw();
    } else {
      gyroFailure = true;
      return 0;
    }
  }
  
  // Returns the odometry calculated x position of the robot in meters.
  public double getXPos() {
    return odometry.getPoseMeters().getX();
  }

  // Returns the odometry calculated y position of the robot in meters.
  public double getYPos() {
    return odometry.getPoseMeters().getY();
  }

  // Resets the robot's odometry to the start point of the path loaded into loadPath()
  public void resetOdometryToPathStart() {
    if (!gyroDisabled && !gyroFailure) {
      PathPlannerState startingState = path.getInitialState();
      odometry.resetPosition(Rotation2d.fromDegrees(getAngPos()), new SwerveModulePosition[] {frontLeftModule.getSMP(), frontRightModule.getSMP(), backRightModule.getSMP(), backLeftModule.getSMP()}, startingState.poseMeters);
    }
  }

  // Resets the robot's odometry pose to x=0, y=0, and heading=0.
  public void resetOdometry() {
    if (!gyroDisabled && !gyroFailure) {
      odometry.resetPosition(Rotation2d.fromDegrees(getAngPos()), new SwerveModulePosition[] {frontLeftModule.getSMP(), frontRightModule.getSMP(), backRightModule.getSMP(), backLeftModule.getSMP()}, new Pose2d());
    }
  }
  
  // Resets the gyro to 0. The current angle of the robot is now defined as 0 degrees. Also clears gyroFailures if a connection is re-established
  public void resetGyro() {
    if (!gyroFailure) {
      gyroFailure = !gyro.isConnected();
      if (gyro.isConnected()) {
        gyro.zeroYaw();
      }
    }
  }

  // Allows the driver to toggle whether the gyro is enabled or disabled. Disabled gyro stops auto and field-oriented control. Useful in case of gyro issues.
  public void toggleGyro() {
    gyroDisabled = !gyroDisabled;
  }

  // The following 4 functions allow the driver to toggle whether each of the swerve modules is on. Useful in the case of an engine failure in match. 
  public void toggleFL() {
    frontLeftModule.toggleModule();
    updateModuleStatus();
  }

  public void toggleFR() {
    frontRightModule.toggleModule();
    updateModuleStatus();
  }

  public void toggleBL() {
    backLeftModule.toggleModule();
    updateModuleStatus();
  }

  public void toggleBR() {
    backRightModule.toggleModule();
    updateModuleStatus();
  }
  
  // Updates moduleError and moduleOffline to reflect the current status of the swerve modules
  private void updateModuleStatus() {
    moduleError = frontLeftModule.moduleFailure || frontRightModule.moduleFailure || backLeftModule.moduleFailure || backRightModule.moduleFailure;
    moduleOffline = frontLeftModule.moduleDisabled || frontRightModule.moduleDisabled || backLeftModule.moduleDisabled || backRightModule.moduleDisabled;
  }
  
  // Publishes all values to Smart Dashboard.
  public void updateDash() {
    SmartDashboard.putNumber("xPos", getXPos());
    SmartDashboard.putNumber("yPos", getYPos());
    SmartDashboard.putNumber("angPos", getAngPos());
    SmartDashboard.putNumber("xVel", xVel);
    SmartDashboard.putNumber("yVel", yVel);
    SmartDashboard.putNumber("angVel", angVel);
    SmartDashboard.putNumber("pathXPos", pathXPos);
    SmartDashboard.putNumber("pathYPos", pathYPos);
    SmartDashboard.putNumber("pathAngPos", pathAngPos);
    SmartDashboard.putNumber("FL angle", frontLeftModule.getAngle());
    SmartDashboard.putNumber("FL pos", frontLeftModule.getPos());
    SmartDashboard.putNumber("FL vel", frontLeftModule.getVel());
    SmartDashboard.putNumber("FL encoder", frontLeftModule.getWheelEncoder());
    SmartDashboard.putNumber("FR angle", frontRightModule.getAngle());
    SmartDashboard.putNumber("FR pos", frontRightModule.getPos());
    SmartDashboard.putNumber("FR vel", frontRightModule.getVel());
    SmartDashboard.putNumber("FR encoder", frontRightModule.getWheelEncoder());
    SmartDashboard.putNumber("BL angle", backLeftModule.getAngle());
    SmartDashboard.putNumber("BL pos", backLeftModule.getPos());
    SmartDashboard.putNumber("BL vel", backLeftModule.getVel());
    SmartDashboard.putNumber("BL encoder", backLeftModule.getWheelEncoder());
    SmartDashboard.putNumber("BR angle", backRightModule.getAngle());
    SmartDashboard.putNumber("BR pos", backRightModule.getPos());
    SmartDashboard.putNumber("BR vel", backRightModule.getVel());
    SmartDashboard.putNumber("BR encoder", backRightModule.getWheelEncoder());
    SmartDashboard.putBoolean("gyroFailure", gyroFailure);
    SmartDashboard.putBoolean("gyroDisabled", gyroDisabled);
    SmartDashboard.putBoolean("moduleErrorFL", frontLeftModule.moduleFailure);
    SmartDashboard.putBoolean("moduleErrorFR", frontRightModule.moduleFailure);
    SmartDashboard.putBoolean("moduleErrorBL", backLeftModule.moduleFailure);
    SmartDashboard.putBoolean("moduleErrorBR", backRightModule.moduleFailure);
    SmartDashboard.putBoolean("moduleError", moduleError);
    SmartDashboard.putBoolean("moduleOfflineFL", frontLeftModule.moduleDisabled);
    SmartDashboard.putBoolean("moduleOfflineFR", frontRightModule.moduleDisabled);
    SmartDashboard.putBoolean("moduleOfflineBL", backLeftModule.moduleDisabled);
    SmartDashboard.putBoolean("moduleOfflineBR", backRightModule.moduleDisabled);
    SmartDashboard.putBoolean("moduleOffline", moduleOffline);
  }
}