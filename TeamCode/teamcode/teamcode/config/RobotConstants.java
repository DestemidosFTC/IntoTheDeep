package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * Classe responsável por agrupar todas as configurações gerais
 * do robô em um só lugar, é recomendado o uso do FTC-Dashboard
 * para modificar e monitorar as mudanças destas informações em
 * tempo real
 */
@Config
public final class RobotConstants {

    // Hubs
    public static final int CONTROLHUB_ID = 0;
    public static final int EXPANSIONHUB_ID = 1;

    // Constraints específicas do Autônomo
    public static TrajectoryVelocityConstraint VEL_CONSTRAINT = DriveConstants.setVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = DriveConstants.setAccelerationConstraint(MAX_ACCEL);

    // Drivetrain
    public static final double CORE_HEX_TICKS_PER_REV = 288.0;
    public static final double HD_HEX_40_TICKS_PER_REV = 1120.0;
    public static final double MECANUM_WHEELS_ANGLE = Math.PI / 4;
    public static double MAX_DRIVETRAIN_POWER = 0.8;

    // Braço
    public static PIDFCoefficients ARM_POSITION_PID = new PIDFCoefficients(0.2,0,0,0);
    public static int ARM_POSITION_TOLERANCE = 0;

    public static double ARM_PID_MAX_POWER_LIMIT = 0.8;
    public static double ARM_PID_MIN_POWER_LIMIT = 0.8;

    public static double ARMS_POWER_SCALE = 0.5;

    public static int ARM_CLOSED_GOAL = 0;
    public static int ARM_LOW_GOAL = 90;

    public static int ARM_MEDIUM_GOAL = 170;
    public static int ARM_HIGH_GOAL = 240;



    public static PIDFCoefficients FOREARM_POSITION_PID = new PIDFCoefficients(0.2,0,0,0);
    public static int FOREARM_POSITION_TOLERANCE = 0 ;

    public static double FOREARM_PID_MAX_POWER_LIMIT = 0.8;
    public static double FOREARM_PID_MIN_POWER_LIMIT = 0.8;

    public static double FOREARM_POWER_SCALE = 0.8;
    public static int FOREARM_CLOSED_GOAL = 0;
    //public static int FOREARM_COLLECT_GOAL = -10;
    public static int FOREARM_LOW_GOAL = 90;
    public static int FOREARM_MEDIUM_GOAL = 170;
    public static int FOREARM_HIGH_GOAL = 240;

    // Servos
    public static double MIN_SERVO_POSITION = 0.35;
    public static double MAX_SERVO_POSITION = 0.7;
    public static final PwmControl.PwmRange MAX_SERVO_RANGE = new PwmControl.PwmRange(600, 2400, 18000);

    // OpenCv
    public static final int IMAGEM_1 = 16;
    public static final int IMAGEM_2 = 14;
    public static final int IMAGEM_3 = 12;

    public static double OPENCV_fx = 850;
    public static double OPENCV_fy = 850;
    public static double OPENCV_cx = (double) RobotConstants.resolutionWidth / 2;
    public static double OPENCV_cy = (double) RobotConstants.resolutionHeight / 2;
    public static double OPENCV_tagsize = 0.06; // em metros
    public static int resolutionWidth = 640;
    public static int resolutionHeight = 480;
}
