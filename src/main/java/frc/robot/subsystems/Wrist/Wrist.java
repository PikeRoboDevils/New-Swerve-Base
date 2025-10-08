// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  WristIO io;
  Elevator Elev;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Pose3d _wristPose;

  /** Creates a new Wrist. */
  public Wrist(WristIO wristIo, Elevator elevator) {
    this.io = wristIo;
    this.Elev = elevator;

    _wristPose =
        new Pose3d(
            new Translation3d(0, 0, 0).plus(Elev.stage3Visuals.getTranslation()),
            new Rotation3d(0, 0, 0));
  }

  @Override
  public void periodic() {

    _wristPose =
        new Pose3d(
            new Translation3d(0.065, 0.0, 0.095).plus(Elev.stage3Visuals.getTranslation()),
            new Rotation3d(
                0, Units.degreesToRadians(io.getAngleDeg() + 90), Units.degreesToRadians(180)));

    Logger.recordOutput("Components/Wrist", _wristPose);

    io.updateInputs(inputs);
    Logger.processInputs("wrist", inputs);
  }

  public Command setAngle(DoubleSupplier angle) {
    return run(() -> io.setAngle(angle.getAsDouble()));
  }

  public Command setVoltage(DoubleSupplier volts) {
    return run(() -> io.setVoltage(volts.getAsDouble()));
  }

  public void disabled() {
    io.setVoltage(0);
  }
}
