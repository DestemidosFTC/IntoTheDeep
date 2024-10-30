package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
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


    // Constantes
    public static final double CORE_HEX_TICKS_PER_REV = 288.0;
    public static final double HD_HEX_40_TICKS_PER_REV = 1120.0;
    public static final double MECANUM_WHEELS_ANGLE = Math.PI / 4;
    public static double MAX_DRIVETRAIN_POWER = 1;

    // Braço
    public static double FeedForward = 0.8;
    public static int ARM_POSITION_TOLERANCE = 0;

    public static double ARM_PID_MAX_POWER_LIMIT = 0.8;
    public static double ARM_PID_MIN_POWER_LIMIT = 0.8;

    public static double ARMS_POWER_SCALE = 0.5;

    public static int ARM_CLOSED_GOAL = 0;
    public static int ARM_LOW_GOAL = 90;
    public static int ARM_MEDIUM_GOAL = 170;
    public static int ARM_HIGH_GOAL = 240;
    public static double ticksToDegrees (double ticks) {
        return (360 * ticks) / CORE_HEX_TICKS_PER_REV;
    }



    public static int FOREARM_POSITION_TOLERANCE = 0 ;

    public static double FOREARM_PID_MAX_POWER_LIMIT = 0.8;
    public static double FOREARM_PID_MIN_POWER_LIMIT = 0.8;

    public static double ARM_POWER_SCALE = 0.8;


    // Servos
    public static double MIN_SERVO_POSITION = 0.7;
    public static double MAX_SERVO_POSITION = 0.45;
    public static final PwmControl.PwmRange MAX_SERVO_RANGE = new PwmControl.PwmRange(600, 2400, 18000);
    public static double degreesToSeconds (double degrees) {
        return (0.24 * degrees) / 60;
    }


    // TensorFlow
    public static final String TFOD_MODEL_ASSET = "model_20240209_153632.tflite";

    public static final String[] LABELS = {
            "Blue TSE",
            "Red TSE"
    };


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
