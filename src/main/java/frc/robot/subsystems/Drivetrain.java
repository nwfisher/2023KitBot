// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.PIDController;
import frc.utils.Conversions;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    /* Motor initialization */
    private WPI_VictorSPX mLeftMotor0;
    private WPI_VictorSPX mLeftMotor1;
    private WPI_VictorSPX mRightMotor0;
    private WPI_VictorSPX mRightMotor1;

    private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    private PIDController mLeftPIDController = new PIDController(Constants.Drivetrain.LEFT_KP, 0, 0);
    private PIDController mRightPIDController = new PIDController(Constants.Drivetrain.RIGHT_KP, 0, 0);

    public Drivetrain() {
        mLeftMotor0 = new WPI_VictorSPX(Constants.Drivetrain.LEFT_MOTOR_O_ID);
        mLeftMotor1 = new WPI_VictorSPX(Constants.Drivetrain.LEFT_MOTOR_1_ID);
        mRightMotor0 = new WPI_VictorSPX(Constants.Drivetrain.RIGHT_MOTOR_O_ID);
        mRightMotor1 = new WPI_VictorSPX(Constants.Drivetrain.RIGHT_MOTOR_1_ID);
    }

    /**
     * Public method to control the drivetrain. Calculates the speed each side of the drivetrain must be at using a ChassisSpeeds object
     * then calls setSpeeds() to set the speed of the drivetrain
     * 
     * @param leftJostickValue Forward/Backward speed taken from left Joystick
     * @param rightJoystickValue Rotational speed (rads/s), taken from right xxsJoystick
     */
    public void drive(double leftJostickValue, double rightJoystickValue) {
        DifferentialDriveWheelSpeeds wheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(leftJostickValue, 0.0, rightJoystickValue));

        setSpeeds(wheelSpeeds);
    }

    /**
     * Private method to set the speeds of our drivetrain. Calculates the needed voltage by using a PID Controller
     *
     * @param wheelSpeeds A DifferentialDriveWheelSpeeds object containing the wheel speeds (in m/s) for each side of the drivetrain
     */
    private void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        double leftOutput = mLeftPIDController.calculate(Conversions.encoderTicksToRPM(mLeftMotor0.getSelectedSensorVelocity(), Constants.Drivetrain.gearRatio), wheelSpeeds.leftMetersPerSecond);
        double rightOutput = mRightPIDController.calculate(Conversions.encoderTicksToRPM(mRightMotor0.getSelectedSensorVelocity(), Constants.Drivetrain.gearRatio), wheelSpeeds.rightMetersPerSecond);

        mLeftMotor0.setVoltage(leftOutput);
        mLeftMotor1.setVoltage(leftOutput);
        mRightMotor0.setVoltage(rightOutput);
        mRightMotor1.setVoltage(rightOutput);
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
