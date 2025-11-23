// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /**
   * We should have a state for every single "pose" the robot will hit. See this document for
   * screenshots of the robot in each state. There are also named positions in cad for each state.
   */
  public enum SuperState {
    IDLE();
    public final Trigger trigger;

    private SuperState() {
      trigger = new Trigger(() -> state == this);
    }

    public Trigger getTrigger() {
      return trigger;
    }
  }

  @AutoLogOutput(key = "Superstructure/State")
  private static SuperState state = SuperState.IDLE;

  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();

  private final SwerveSubsystem swerve;
  private final CommandXboxControllerSubsystem driver;
  private final CommandXboxControllerSubsystem operator;

  // Declare triggers
  @AutoLogOutput(key = "Superstructure/Pre Score Request")
  public Trigger preScoreReq;

  @AutoLogOutput(key = "Superstructure/Score Request")
  public Trigger scoreReq;

  // @AutoLogOutput(key = "Superstructure/At Extension?")
  // public Trigger atExtensionTrigger = new Trigger(this::atExtension).or(Robot::isSimulation);

  /** Creates a new Superstructure. */
  public Superstructure(
      SwerveSubsystem swerve,
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator) {
    this.swerve = swerve;
    this.driver = driver;
    this.operator = operator;

    addTriggers();
    addTransitions();

    stateTimer.start();
  }

  private void addTriggers() {
    preScoreReq = driver.rightTrigger().or(Autos.autoPreScoreReq);

    scoreReq = driver.rightTrigger().negate().and(DriverStation::isTeleop).or(Autos.autoScoreReq);
  }

  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
    Logger.recordOutput("Superstructure/State Timer", stateTimer.get());
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger) {
    // when 1) the robot is in the start state and 2) the trigger is true, the robot changes state
    // to the end state
    trigger.and(start.getTrigger()).onTrue(changeStateTo(end));
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   * @param cmd some command to run while making the transition
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger, Command cmd) {
    // when 1) the robot is in the start state and 2) the trigger is true, the robot changes state
    // to the end state IN PARALLEL to running the command that got passed in
    trigger.and(start.getTrigger()).onTrue(Commands.parallel(changeStateTo(end), cmd));
  }

  // public boolean atExtension(SuperState state) {
  // }

  // public boolean atExtension() {
  //   return atExtension(state);
  // }

  private Command changeStateTo(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state from " + state + " to " + nextState);
              stateTimer.reset();
              this.prevState = state;
              state = nextState;
              setSubstates();
            })
        .ignoringDisable(true)
        .withName("State Change Command");
  }

  private void setSubstates() {}

  // public Command transitionAfterZeroing() {
  //  }

  private void addTransitions() {}

  /**
   * <b>Only for setting initial state at the beginning of auto</b>
   *
   * @param state the state to set to
   */
  public void resetStateForAuto(SuperState nextState) {
    System.out.println("Resetting state from " + state + " to " + nextState + " for auto.");
    stateTimer.reset();
    this.prevState = state;
    state = nextState;
    setSubstates();
  }

  public static SuperState getState() {
    return state;
  }

  public boolean stateIsIdle() {
    return getState() == SuperState.IDLE;
  }
}
