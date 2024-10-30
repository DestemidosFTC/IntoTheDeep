package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Subsistema responsável pelo mecanismo de coleta (Intake),
 * inspirado no paradigma de Command-Based Programming, da FRC.
 */
public class ServoSystem implements Subsystem {
    public final ServoImplEx wristServoPlane;
    public final ServoImplEx wristServoA;
    public final ServoImplEx wristServoB;
    public final CRServoImplEx wristServoC;
    public final CRServoImplEx wristServoD;
    public final CRServoImplEx wristServoE;
    public final CRServoImplEx wristServoF;
    private final ElapsedTime fistTimer;

    /**
     * Construtor padrão que configura os servos do sistema
     * @param hardwareMap presente em todo OpMode
     */
    public ServoSystem(HardwareMap hardwareMap) {
        wristServoA = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_d"); //Porta 0, ExpansionHub
        wristServoB = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_e"); //Porta 2, ExpansionHub
        //definição de motores da garra

        wristServoC = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoD"); //Porta 3, ExpansionHub
        wristServoD = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoE"); //Porta 4, ExpansionHub
        //definição de motores do punho

        wristServoE = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "hook1"); //Porta 1, ControlHub
        wristServoF = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "hook2"); //Porta 2, ControlHub

        wristServoPlane = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_plane"); //Porta 5, ExpansionHub
        //definição do servo do lançador avião

        wristServoB.setDirection(ServoImplEx.Direction.REVERSE);
        wristServoD.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServoF.setDirection(DcMotorSimple.Direction.REVERSE);

        //Revertendo os valores para os motores realizarem movimentos opostos no punho e na garra

        fistTimer = new ElapsedTime();
        fistTimer.reset();

    }

    public void openWrist() {
        wristServoA.setPosition(RobotConstants.MIN_SERVO_POSITION);
        wristServoB.setPosition(RobotConstants.MIN_SERVO_POSITION);
    }

    public void closeWrist() {
        wristServoA.setPosition(RobotConstants.MAX_SERVO_POSITION);
        wristServoB.setPosition(RobotConstants.MAX_SERVO_POSITION);
    }

    public void moveFist(double power) {
        wristServoC.setPower(power);
        wristServoD.setPower(power);
    }

    public void moveHook(double power) {
        wristServoE.setPower(power);
        wristServoF.setPower(power);
    }

    public void launchDrone() {
        wristServoPlane.setPosition(0.2);
    }

    public void fistServoRotation(double degrees) {

        fistTimer.reset();

        double time = RobotConstants.degreesToSeconds(degrees);
        int power = 1;

        if(time < 0) {
            power = -1;
            time = time * -1;
        }

        while (fistTimer.seconds() < time) {
            moveFist(power);
        }

        moveFist(0);
        fistTimer.reset();

    }


}

