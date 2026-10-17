// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeDeploymentConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

public class LowerIntakeCommand extends Command {
  private final IntakeDeploymentSubsystem m_intakeDeploymentSubsystem;
  private boolean m_limitSwitchHit = false;
  private double m_targetPosition = 0.0;
  private double m_limitHitTimestamp = 0.0;

  /** Creates a new LowerIntakeCommand. */
  public LowerIntakeCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_intakeDeploymentSubsystem = intakeDeploymentSubsystem;
    addRequirements(m_intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limitSwitchHit = false;
    m_targetPosition = 0.0;
    m_limitHitTimestamp = 0.0;
    m_intakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentDownRPM);
    RobotContainer.setRaisingIntake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_limitSwitchHit) {
      if (m_intakeDeploymentSubsystem.isRevLimitSwitchPressed()) {
        m_limitSwitchHit = true;
        m_limitHitTimestamp = Timer.getFPGATimestamp();
        // Downward motion is negative, so subtract extra rotations to continue further down:
        m_targetPosition = m_intakeDeploymentSubsystem.getPosition() - IntakeDeploymentConstants.kIntakeDeploymentExtraRotations;
        
        // Actively drive down with controlled speed to force through mechanical backlash/slop:
        m_intakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentSlowDownRPM);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeDeploymentSubsystem.stop();
    // Zero the encoder at the fully lowered / seated position
    if (!interrupted && m_limitSwitchHit) {
      m_intakeDeploymentSubsystem.resetEncoder(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_limitSwitchHit) {
      return false;
    }
    double timeSinceHit = Timer.getFPGATimestamp() - m_limitHitTimestamp;

    // 1. Finished when the motor completes the extra rotations:
    boolean reachedTarget = m_intakeDeploymentSubsystem.getPosition() <= m_targetPosition;

    // 2. True stall detection: only if the motor has physically stopped rotating (< 40 RPM)
    // and is pulling high current against the hard mechanical stop.
    boolean stallDetected = (timeSinceHit > 0.15) 
        && (Math.abs(m_intakeDeploymentSubsystem.getRPM()) < 40.0)
        && (m_intakeDeploymentSubsystem.getOutputCurrent() >= IntakeDeploymentConstants.kIntakeDeploymentStallCurrentAmps);

    // 3. Safety timeout:
    boolean timedOut = timeSinceHit >= IntakeDeploymentConstants.kIntakeDeploymentLockdownTimeout;

    return reachedTarget || stallDetected || timedOut;
  }
}
