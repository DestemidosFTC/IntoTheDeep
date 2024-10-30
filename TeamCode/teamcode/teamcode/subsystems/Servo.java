package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Subsistema responsável pelo mecanismo de coleta (Intake),
 * inspirado no paradigma de Command-Based Programming, da FRC.
 */
public class Servo implements Subsystem {
    //public final DcMotorEx gripper;
    public final ServoImplEx wristServoA;
    public final ServoImplEx wristServoB;
    public final CRServoImplEx wristServoC;
    public final CRServoImplEx wristServoD;
    private final ElapsedTime fistTimer;
    /**
     * Construtor padrão que configura os servos do sistema
     * @param hardwareMap presente em todo OpMode
     */
    public Servo(HardwareMap hardwareMap) {
        wristServoA = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_d"); //Porta 0, Servo
        wristServoB = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_e"); //Porta 2, Servo
        //definição de motores da garra

        wristServoC = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoD"); //Porta 3, Servo
        wristServoD = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoE"); //Porta 4, Servo
        //definição de motores do punho

        wristServoB.setDirection(ServoImplEx.Direction.REVERSE);
        wristServoD.setDirection(DcMotorSimple.Direction.REVERSE);

        //Revertendo os valores para os motores realizarem movimentos opostos no punho e na garra

        fistTimer = new ElapsedTime();
        fistTimer.reset();
    }

    public void openWrist() {
        wristServoA.setPosition(RobotConstants.MAX_SERVO_POSITION);
        wristServoB.setPosition(RobotConstants.MAX_SERVO_POSITION);
    }

    public void closeWrist() {
        wristServoA.setPosition(RobotConstants.MIN_SERVO_POSITION);
        wristServoB.setPosition(RobotConstants.MIN_SERVO_POSITION);
    }


    public void moveMonheca(double power) {
        wristServoC.setPower(power);
        wristServoD.setPower(power);
        //variavél para definir a força feita para mover a monheca
    }

    public void turnOffMonheca() {
        wristServoC.setPwmDisable();
        wristServoD.setPwmDisable();

    }


    public void fistServoRotation(double rotation) {

        //reseta o temporizador
        fistTimer.reset();

        //define as variáveis que utilizaremos nesse void
        int power = -1;
        double timer = rotation / 225;

        //verifica se as rotações estão negativas.
        if(timer < 0) {
            power = 1;
            timer = timer * -1;
        }

        //loop para executar pela quantidade de tempo
        while (fistTimer.seconds() < timer) {
            wristServoC.setPower(power);
            wristServoD.setPower(power);
        }

        //desliga os servos e reinicia o temporizador
        turnOffMonheca();
        fistTimer.reset();
    }

    }

