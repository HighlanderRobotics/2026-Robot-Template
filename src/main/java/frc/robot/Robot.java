// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread;
import frc.robot.utils.CommandXboxControllerSubsystem;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.REPLAY;
  public static final boolean TUNING_MODE = true;
  public boolean hasZeroedSinceStartup = false;

  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  private Alert manualArmRezeroAlert;
  private Alert driverJoystickDisconnectedAlert;
  private Alert operatorJoystickDisconnectedAlert;

  private static CANBus canivore = new CANBus("*");

  private static CANBusStatus canivoreStatus = canivore.getStatus();

  // Instantiate subsystems

  // Maple Sim Stuff
  private final DriveTrainSimulationConfig driveTrainSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          // TODO: MAKE SURE THIS MODULE IS CORRECT
          .withSwerveModule(
              COTS.ofMark4n(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX60Foc(1),
                  // Still not sure where the 1.5 came from
                  1.5,
                  // Running l2+ swerve modules
                  2))
          .withTrackLengthTrackWidth(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthX()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthY()))
          .withBumperSize(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperWidth()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperLength()))
          .withRobotMass(SwerveSubsystem.SWERVE_CONSTANTS.getMass());

  private final SwerveDriveSimulation swerveSimulation =
      new SwerveDriveSimulation(driveTrainSimConfig, new Pose2d(3, 3, Rotation2d.kZero));
  // Subsystem initialization
  private final SwerveSubsystem swerve = new SwerveSubsystem(swerveSimulation);
  private final LEDSubsystem leds = new LEDSubsystem(new LEDIOReal());

  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  @AutoLogOutput(key = "Superstructure/Autoaim Request")
  private Trigger autoAimReq = driver.rightBumper().or(driver.leftBumper());

  @AutoLogOutput(key = "Robot/Pre Zeroing Request")
  private Trigger preZeroingReq = driver.a();

  @AutoLogOutput(key = "Robot/Zeroing Request")
  private Trigger zeroingReq = driver.b();

  private final Superstructure superstructure = new Superstructure(swerve, driver, operator);

  private final Autos autos;
  private Optional<Alliance> lastAlliance = Optional.empty();
  @AutoLogOutput boolean haveAutosGenerated = false;
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

  // Logged mechanisms

  @SuppressWarnings("resource")
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(6.0);
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "2026 Template");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_TYPE.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (ROBOT_TYPE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    Logger.recordOutput("Canivore Status", canivoreStatus.Status);

    PhoenixOdometryThread.getInstance().start();

    // Set default commands

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSimulation);
    }

    swerve.setDefaultCommand(
        swerve.driveOpenLoopFieldRelative(
            () ->
                new ChassisSpeeds(
                        modifyJoystick(driver.getLeftY())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getRightX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
                    .times(-1)));

    addControllerBindings();

    autos = new Autos(swerve, superstructure::resetStateForAuto);
    autoChooser.addDefaultOption("None", Commands.none());

    // Generates autos on connected
    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    && DriverStation.getAlliance().isPresent()
                    && !haveAutosGenerated)
        .onTrue(Commands.print("Connected"))
        .onTrue(Commands.runOnce(this::addAutos).ignoringDisable(true));

    new Trigger(
            () -> {
              boolean allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(Commands.runOnce(this::addAutos).ignoringDisable(true));

    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              System.out.println("Interrupted: " + interrupted);
              System.out.println(
                  "Interrputing: "
                      + (interrupting.isPresent() ? interrupting.get().getName() : "none"));
            });

    // Add autos on alliance change
    new Trigger(
            () -> {
              var allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(
                    leds.setBlinkingCmd(() -> Color.kWhite, () -> Color.kBlack, 20.0)
                        .withTimeout(1.0))
                .ignoringDisable(true));

    // Add autos when first connecting to DS
    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    && DriverStation.getAlliance().isPresent()
                    && !haveAutosGenerated) // TODO check that the haveautosgenerated doesn't break
        // anything?
        .onTrue(Commands.print("connected"))
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(
                    leds.setBlinkingCmd(() -> Color.kWhite, () -> Color.kBlack, 20.0)
                        .withTimeout(1.0))
                .ignoringDisable(true));
    SmartDashboard.putData("Add autos", Commands.runOnce(this::addAutos).ignoringDisable(true));

    manualArmRezeroAlert =
        new Alert(
            "Arm has been manually rezeroed at least once this match. Arm cancoder may not be working!",
            AlertType.kWarning);

    driverJoystickDisconnectedAlert =
        new Alert("Driver controller disconnected!", AlertType.kError);
    operatorJoystickDisconnectedAlert =
        new Alert("Operator controller disconnected!", AlertType.kError);

    Logger.recordOutput(
        "test",
        new Pose2d(new Translation2d(), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(45.0))));
  }

  private TalonFXConfiguration createRollerConfig(
      InvertedValue inverted,
      double currentLimit,
      double sensorToMechanismRatio,
      double kS,
      double kV,
      double kP) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;

    config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

    return config;
  }

  private TalonFXConfiguration createPivotConfig(
      InvertedValue inverted,
      double supplyCurrentLimit,
      double statorCurrentLimit,
      double sensorToMechRatio,
      double kV,
      double kG,
      double kS,
      double kP,
      double kI,
      double kD) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kV = kV;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = sensorToMechRatio;

    return config;
  }

  private CANcoderConfiguration createCANcoderConfig(
      SensorDirectionValue directionValue,
      double MagnetOffset,
      double AbsoluteSensorDiscontinuityPoint) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = directionValue;
    config.MagnetSensor.MagnetOffset = MagnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint;

    return config;
  }

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }

  private void addControllerBindings() {
    // heading reset
    driver
        .leftStick()
        .and(driver.rightStick())
        .onTrue(
            Commands.runOnce(
                () ->
                    swerve.setYaw(
                        DriverStation.getAlliance().equals(Alliance.Blue)
                            // ? Rotation2d.kCW_90deg
                            // : Rotation2d.kCCW_90deg)));
                            ? Rotation2d.kZero
                            : Rotation2d.k180deg)));

    // ---zeroing stuff---

    new Trigger(() -> DriverStation.isJoystickConnected(0))
        .negate()
        .onTrue(Commands.runOnce(() -> driverJoystickDisconnectedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> driverJoystickDisconnectedAlert.set(false)));

    new Trigger(() -> DriverStation.isJoystickConnected(1))
        .negate()
        .onTrue(Commands.runOnce(() -> operatorJoystickDisconnectedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> operatorJoystickDisconnectedAlert.set(false)));
  }

  private void addAutos() {
    System.out.println("------- Regenerating Autos");
    System.out.println(
        "Regenerating Autos on " + DriverStation.getAlliance().map((a) -> a.toString()));
    haveAutosGenerated = true;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.periodic();

    // Log mechanism poses
  }

  @Override
  public void simulationInit() {
    // Sets the odometry pose to start at the same place as maple sim pose
    swerve.resetPose(swerveSimulation.getSimulatedDriveTrainPose());
  }

  @Override
  public void simulationPeriodic() {
    // Update maple simulation
    SimulatedArena.getInstance().simulationPeriodic();
    // Log simulated pose
    Logger.recordOutput("MapleSim/Pose", swerveSimulation.getSimulatedDriveTrainPose());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
