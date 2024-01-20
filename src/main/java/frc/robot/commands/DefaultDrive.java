// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public DefaultDrive() {
		addRequirements(SWERVE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Translation2d moveVelocity = InputManager.getInstance().getSwerveVelocity2D();
		double xVelocity = moveVelocity.getX();
		double yVelocity = -moveVelocity.getY();
		double rotationalVelocity = InputManager.getInstance().getSwerveRotationalVelocity();
		rotationalVelocity = MathUtil.applyDeadband(rotationalVelocity, 0.1);
		double speed = Math.hypot(xVelocity, yVelocity);
		double deadbandSpeed = MathUtil.applyDeadband(speed, 0.1);
		double velocityDir = Math.atan2(yVelocity, xVelocity);
		double sign = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 1.0 : -1.0);

		xVelocity = cos(velocityDir) * deadbandSpeed * maxSpeed * speedMultiplier * sign;
		yVelocity = sin(velocityDir) * deadbandSpeed * maxSpeed * speedMultiplier * sign;
		rotationalVelocity = rotationalVelocity * angularSpeed * angularMultiplier;

		SWERVE.drive(xVelocity, yVelocity, rotationalVelocity);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
