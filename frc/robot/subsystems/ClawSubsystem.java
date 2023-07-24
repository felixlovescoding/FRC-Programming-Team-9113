// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Stack;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.OpenClawCommand;

public class ClawSubsystem extends SubsystemBase {
  public static final int CLAW_LEFT = 0;
  public static final int CLAW_RIGHT = 1;

  CANSparkMax leftClaw = new CANSparkMax(CLAW_LEFT, MotorType.kBrushless);
  CANSparkMax rightClaw = new CANSparkMax(CLAW_RIGHT, MotorType.kBrushless);

  Deque<CommandBase> actions;
  CommandBase current;
  boolean inProcess;

  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem() {
    actions = new ArrayDeque<>();
  }

  public static final Integer ARBITARY_CLAW_ENCODER_DISTANCE = 90;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (current == null) {
      current = actions.poll();
    }

    if (current == null)
      return;

    int openFactor = current instanceof OpenClawCommand ? 1 : -1;

    if (inProcess) {

      double angle = leftClaw.getEncoder().getPosition();

      if (angle < ARBITARY_CLAW_ENCODER_DISTANCE) {
        leftClaw.set(openFactor * 1);
        rightClaw.set(openFactor * 1);
      } else {
        inProcess = false;
        leftClaw.set(0);
        rightClaw.set(0);

        leftClaw.getEncoder().setPosition(0);

        current = null;
      }

    } else {
      inProcess = true;

      double angle = leftClaw.getEncoder().getPosition();

      leftClaw.set(openFactor * 1);
      rightClaw.set(openFactor * 1);

    }
  }

  @Override
  public void simulationPeriodic() {
    leftClaw.set(1.0);

    while (leftClaw.getEncoder().getPosition() < 5) {
      try {
        wait(2);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    leftClaw.set(0);

    rightClaw.set(1.0);

    while (rightClaw.getEncoder().getPosition() < 5) {
      try {
        wait(2);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    rightClaw.set(0);
    // This method will be called once per scheduler run during simulation
    leftClaw.set(-1.0);

    while (leftClaw.getEncoder().getPosition() > 0) {

    }

    leftClaw.set(0);

    rightClaw.set(-1.0);

    while (rightClaw.getEncoder().getPosition() > 0) {

    }

    rightClaw.set(0);
  }

  public Stack<CommandBase> commands() {
    return actions;
  }

}
