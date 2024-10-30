package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.DriveConstants;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.utils.UnitConversion;

import java.util.ArrayList;
import java.util.List;

public class AutonomoSystem extends MecanumDrive {
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final TrajectoryFollower follower;
    private final Drivetrain drivetrain;
    private final LocalizationSystem localizationSystem;
    private final VoltageSensor batteryVoltageSensor;

    public AutonomoSystem(Drivetrain drive, LocalizationSystem localize, VoltageSensor batteryVoltageSensor) {
        super(DriveConstants.kV,
                DriveConstants.kA,
                DriveConstants.kStatic,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.WHEEL_BASE,
                DriveConstants.LATERAL_MULTIPLIER);

        drivetrain = drive;
        localizationSystem = localize;
        this.batteryVoltageSensor = batteryVoltageSensor;

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        follower = new HolonomicPIDVAFollower(
                DriveConstants.AXIAL_PID,
                DriveConstants.LATERAL_PID,
                DriveConstants.HEADING_PID,
                new Pose2d(0.25, 0.25, Math.toRadians(3.0)),
                0.5);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, DriveConstants.HEADING_PID);
    }

    @Override
    protected double getRawExternalHeading() {
        return localizationSystem.getRobotHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return drivetrain.getRawWheelsPositions();
    }
    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : drivetrain.getMotors()) {
            wheelVelocities.add(UnitConversion.encoderTicksToInches(motor.getVelocity(), DriveConstants.TICKS_PER_REV, TRACK_WIDTH));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drivetrain.setMotorsPower(
                v3,  // direita frente
                v2,  // direita trás
                v,   // esquerda frente
                v1); // esquerda trás
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : drivetrain.getMotors()) {
            motor.setMode(runMode);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : drivetrain.getMotors()) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, RobotConstants.VEL_CONSTRAINT, RobotConstants.ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, RobotConstants.VEL_CONSTRAINT, RobotConstants.ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, RobotConstants.VEL_CONSTRAINT, RobotConstants.ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                RobotConstants.VEL_CONSTRAINT, RobotConstants.ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }
}
