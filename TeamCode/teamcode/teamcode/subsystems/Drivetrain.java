package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConstants.TRACK_WIDTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.DriveConstants;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.UnitConversion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Subsistema responsável por gerenciar e comandar o chassis
 * do robô e seus motores associados.
 */
public final class Drivetrain {

    /*
     * (ramalho): segue abaixo o padrão de input pra qualquer função
     * que utilize os motores da base:
     *
     *      0 - Motor Direita-Frente
     *      1 - Motor Direita-Trás
     *      2 - Motor Esquerda-Frente
     *      3 - Motor Esquerda-Trás
     *
     */

    private final List<DcMotorEx> motors;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorDireitaTras;
    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private double batteryVoltage = 12.0;

    /**
     * Construtor padrão que configura o sistema de motores
     * acessando o {@link HardwareMap}
     *
     * @param hardwareMap presente em todo OpMode
     */
    public Drivetrain(HardwareMap hardwareMap) {
        motorDireitaFrente = hardwareMap.get(DcMotorEx.class, "DF"); // porta 0 - controlHub
        motorDireitaTras = hardwareMap.get(DcMotorEx.class, "DT"); // porta 1 - controlHub
        motorEsquerdaFrente = hardwareMap.get(DcMotorEx.class, "EF"); // porta  - controlhub
        motorEsquerdaTras = hardwareMap.get(DcMotorEx.class, "ET"); // porta 3 - controlHub

        // precisamos inverter apenas os motores da esquerda
        motorEsquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(motorDireitaFrente, motorDireitaTras, motorEsquerdaFrente, motorEsquerdaTras);

        resetEncoderWheels();

        configRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        configZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // configurando alguns detalhes ai blz
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);

            motor.setMotorType(motorConfigurationType);
        }
    }

    /**
     * Define o modo de operação de todos os motores
     *
     * @param runMode modo de operação
     */
    public void configRunMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    /**
     * Configura o comportamento dos motores, quando sua força
     * for equivalente a zero.
     *
     * @param ZeroPowerBehavior tipo de comportamento
     */
    public void configZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior);
        }
    }

    /**
     * Reinicia a contagem absoluta dos encoders
     */
    public void resetEncoderWheels() {
        configRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Acessa os 4 motores utilizando uma única lista.
     *
     * @return Lista dos Motores
     */
    public List<DcMotorEx> getMotors() {
        return motors;
    }

    /**
     * Acessa os motores através de seus ID's na lista
     *
     * @param ID número da porta conectada ao motor em questão.
     * @return o motor correspondente ao ID
     */
    public DcMotorEx getMotor(int ID) {
        return motors.get(ID);
    }

    public void updateVoltage(double volts) {
        batteryVoltage = volts;
    }

    /**
     * Define uma força para cada motor de forma independente
     */
    public void setMotorsPower(double DireitaFrentePower, double DireitaTrasPower, double EsquerdaFrentePower, double EsquerdaTrasPower) {
        double scalar = 12.0 / batteryVoltage;

        motorDireitaFrente.setPower(
                Range.clip(DireitaFrentePower * scalar,
                        -RobotConstants.MAX_DRIVETRAIN_POWER,
                        RobotConstants.MAX_DRIVETRAIN_POWER)
        );
        motorDireitaTras.setPower(
                Range.clip(DireitaTrasPower * scalar,
                        -RobotConstants.MAX_DRIVETRAIN_POWER,
                        RobotConstants.MAX_DRIVETRAIN_POWER)
        );
        motorEsquerdaFrente.setPower(
                Range.clip(EsquerdaFrentePower * scalar,
                        -RobotConstants.MAX_DRIVETRAIN_POWER,
                        RobotConstants.MAX_DRIVETRAIN_POWER)
        );
        motorEsquerdaTras.setPower(
                Range.clip(EsquerdaTrasPower * scalar,
                        -RobotConstants.MAX_DRIVETRAIN_POWER,
                        RobotConstants.MAX_DRIVETRAIN_POWER)
        );
    }

    /**
     * Define uma força para todos os motores, de forma uniforme
     *
     * @param power valor da força, de -1.0 a 1.0
     */
    public void setAllPower(double power) {
        double scalar = 12.0 / batteryVoltage;

        motorDireitaFrente.setPower(power * scalar);
        motorDireitaTras.setPower(power * scalar);
        motorEsquerdaFrente.setPower(power * scalar);
        motorEsquerdaTras.setPower(power * scalar);
    }

    /**
     * Retorna as velocidades dos motores em lista
     */
    public List<Double> getRawWheelsVelocities() {
        List<Double> velocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            velocities.add(motor.getVelocity());
        }
        return velocities;
    }

    /**
     * Retorna a posição dos motores em lista
     */
    public List<Double> getRawWheelsPositions() {
        List<Double> positions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            positions.add((double) motor.getCurrentPosition());
        }
        return positions;
    }

    /**
     * Estilo de controle extremamente simples, onde só há movimentos frontais
     * e de rotação, ideal para drivetrains que não usam as rodas mecanum
     *
     * @param driver gamepad do jogador
     */
    public void tankController(Gamepad driver) {
        double joystick_y = -driver.left_stick_y;
        double giro = driver.right_stick_x;

        double direitaFrentePower = (joystick_y - giro);
        double direitaTrasPower = (joystick_y - giro);
        double esquerdaFrentePower = (joystick_y + giro);
        double esquerdaTrasPower = (joystick_y + giro);

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }

    /**
     * Movimentação padrão e mais refinada do nosso controle, baseada no vídeo
     * extremamente didático do Gavin Ford:
     * <a href="https://youtu.be/gnSW2QpkGXQ">How to Use Mecanum Wheels in 200 Seconds</a>
     *
     * @param driver gamepad do jogador
     */
    public void standardMecanumController(Gamepad driver) {
        double joystick_y = -driver.left_stick_y;
        double joystick_x = driver.left_stick_x;
        double giro = -driver.right_stick_x;

        double direitaFrentePower = (joystick_y - joystick_x - giro);
        double direitaTrasPower = (joystick_y + joystick_x - giro);
        double esquerdaFrentePower = (joystick_y + joystick_x + giro);
        double esquerdaTrasPower = (joystick_y - joystick_x + giro);

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }

    /**
     * Estilo de controle mais preciso, baseado no 2° método apresentado pelo youtuber Gavin Ford:
     * <a href="https://youtu.be/gnSW2QpkGXQ">How to Use Mecanum Wheels in 200 Seconds</a>
     *
     * @param driver gamepad do jogador
     */
    public void preciseMecanumController(Gamepad driver) {

        // principais inputs do controle
        double theta = Math.atan2(-driver.left_stick_y, driver.left_stick_x);       // ângulo do joystick
        double direction = Math.hypot(driver.left_stick_x, -driver.left_stick_y);   // direção que o joystick aponta
        double turn = -driver.right_stick_x;                                        // giro

        // usamos o seno e o cosseno para controlar os pares de rodas nas diagonais
        // para entender melhor o do porquê deste cálculo, veja o vídeo referênciado
        double sen = Math.sin(theta - RobotConstants.MECANUM_WHEELS_ANGLE);
        double cos = Math.cos(theta - RobotConstants.MECANUM_WHEELS_ANGLE);

        // neste caso, o denominador vai "normalizar" os valores de cada eixo
        // utilizando o maior valor entre o seno e o cosseno
        double denominador = Math.max(Math.abs(sen), Math.abs(cos));

        // aplicamos o movimento final
        double direitaFrentePower = direction * (sen / denominador) - turn;
        double direitaTrasPower = direction * (cos / denominador) - turn;
        double esquerdaFrentePower = direction * (cos / denominador) + turn;
        double esquerdaTrasPower = direction * (sen / denominador) + turn;

        if (direction + Math.abs(turn) > 1) {
            direitaFrentePower /= direction + Math.abs(turn);
            direitaTrasPower /= direction + Math.abs(turn);
            esquerdaFrentePower /= direction + Math.abs(turn);
            esquerdaTrasPower /= direction + Math.abs(turn);
        }

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }

    /**
     *
     * @param driver
     */
    public void alternativeMecanumController(Gamepad driver) {
        double direction = Math.hypot(driver.left_stick_x, -driver.left_stick_y);
        double robotAngle = Math.atan2(-driver.left_stick_y, driver.left_stick_x) - RobotConstants.MECANUM_WHEELS_ANGLE;
        double rightX = driver.right_stick_x;

        final double v1 = direction * Math.cos(robotAngle) + rightX;
        final double v2 = direction * Math.sin(robotAngle) - rightX;
        final double v3 = direction * Math.sin(robotAngle) + rightX;
        final double v4 = direction * Math.cos(robotAngle) - rightX;

        setMotorsPower(v2, v4, v1, v3);
    }

    /**
     * Estilo de controle que sempre direciona o robô numa posição relativa ao jogador,
     * independente da orentação do robô na arena
     *
     * @param driver     gamepad do jogador
     * @param botHeading ângulo atual do robô
     */
    public void fieldOrientedController(Gamepad driver, double botHeading) {
        double joystick_y = -driver.left_stick_y;
        double joystick_x = driver.left_stick_x;
        double giro = driver.right_stick_x;

        double rotationX = joystick_x * Math.cos(botHeading) - joystick_y * Math.sin(botHeading);
        double rotationY = joystick_x * Math.sin(botHeading) + joystick_y * Math.cos(botHeading);

        double denominador = Math.max(Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        double direitaFrentePower = (rotationY - rotationX - giro) / denominador;
        double direitaTrasPower = (rotationY + rotationX - giro) / denominador;
        double esquerdaFrentePower = (rotationY + rotationX + giro) / denominador;
        double esquerdaTrasPower = (rotationY - rotationX + giro) / denominador;

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }
}