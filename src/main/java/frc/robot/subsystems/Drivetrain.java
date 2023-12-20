// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.PIDController;
import frc.utils.Conversions;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    /* Motor initialization */
    private VictorSPX mLeftMotor0;
    private VictorSPX mLeftMotor1;
    private VictorSPX mRightMotor0;
    private VictorSPX mRightMotor1;

    private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    private PIDController mLeftPIDController = new PIDController(Constants.Drivetrain.LEFT_KP, 0, 0);
    private PIDController mRightPIDController = new PIDController(Constants.Drivetrain.RIGHT_KP, 0, 0);

    public Drivetrain() {
        mLeftMotor0 = new VictorSPX(Constants.Drivetrain.LEFT_MOTOR_O_ID);
        mLeftMotor1 = new VictorSPX(Constants.Drivetrain.LEFT_MOTOR_1_ID);
        mRightMotor0 = new VictorSPX(Constants.Drivetrain.RIGHT_MOTOR_O_ID);
        mRightMotor1 = new VictorSPX(Constants.Drivetrain.RIGHT_MOTOR_1_ID);
    }

    public void drive(double leftJostickValue, double rightJoystickValue) {
        DifferentialDriveWheelSpeeds wheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(leftJostickValue, 0.0, rightJoystickValue));
    }

    private void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        double leftOutput = mLeftPIDController.calculate(Conversions.encoderTicksToRPM(mLeftMotor0.getSelectedSensorVelocity(), Constants.Drivetrain.geatRatio), wheelSpeeds.leftMetersPerSecond);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
