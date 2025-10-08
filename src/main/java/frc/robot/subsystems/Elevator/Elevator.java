// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public Pose3d stage1Visuals, stage2Visuals, stage3Visuals = new Pose3d();

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // TODO: Remove all "/13.5" once absolute encoder is added
    stage1Visuals =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(3.55),
                0,
                (io.getPosition() / 3) / 13.5 + Units.inchesToMeters(5.6)),
            new Rotation3d(Units.degreesToRadians(90), 0, Units.degreesToRadians(90)));
    stage2Visuals =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(3.55),
                0,
                (io.getPosition() / 1.5 / 13.5) + Units.inchesToMeters(6.6)),
            new Rotation3d(Units.degreesToRadians(90), 0, Units.degreesToRadians(90)));
    stage3Visuals =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(3.55),
                0,
                (io.getPosition() / 13.5) + Units.inchesToMeters(7.6)),
            new Rotation3d(Units.degreesToRadians(90), 0, Units.degreesToRadians(90)));

    Logger.recordOutput("Components/Stage1", stage1Visuals);
    Logger.recordOutput("Components/Stage2", stage2Visuals);
    Logger.recordOutput("Components/Stage3", stage3Visuals);
  }

  public Command setPoint(DoubleSupplier position) {
    return run(() -> io.setPosition(position.getAsDouble()));
  }

  public Command setVoltage(DoubleSupplier volts) {
    return run(() -> io.setVoltage(volts.getAsDouble())).finallyDo(() -> io.setVoltage(0));
  }

  public void disabled() {
    io.setPosition(io.getPosition());
  }

  public void reset() {
    io.setEncoderPosition(new Rotation2d(0));
  }
}
