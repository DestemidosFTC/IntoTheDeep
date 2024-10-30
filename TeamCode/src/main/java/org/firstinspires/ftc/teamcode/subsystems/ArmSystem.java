package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Subsistema responsável pelo controle do braço
 * seja de forma precisa, utilizando o Controlador PID,
 * ou de forma manual pelo próprio jogador
 */
public final class ArmSystem implements Subsystem {

    // Motores
    public final DcMotorEx leftarmMotor;
    public final DcMotorEx rightarmMotor;
    
    private static final double KP = 0.1;
    private static final double KI = 0.0;
    private static final double KD = 0.0015;
    private static final double ZERO_OFFSET = 55;
    private static final double MIN_POS = ZERO_OFFSET;
    private static final double MAX_POS = 205;
    private static final double ENCODER_CPR = 288.0;
    private static final double GEAR_RATIO = 1;

    private static final double TICKS_PER_DEGREE = ENCODER_CPR * GEAR_RATIO / 360.0;

    private double setTargetPosinDegrees;
    private double revPowerLimit, fwdPowerLimit;
    //
    private double robotVoltage = 12.0;

    //
    private int armTarget;
    private int forearmTarget;

    public double forearmPID;
    private double armPID;

    private double armFeedforward;
    public double forearmFeedforward;

    // Controlador PID pros motores
    public final PIDController armController;

    //
    public enum ArmStage {
        CLOSED,
        LOW,
        MEDIUM,
        HIGH
    }


    public enum ForearmStage {
        CLOSED,
        LOW,
        MEDIUM,
        HIGH
    }

    /**
     * Construtor padrão para a configuração do hardware
     * @param hardwareMap presente em todo OpMode
     */
    public ArmSystem(HardwareMap hardwareMap) {
        leftarmMotor = hardwareMap.get(DcMotorEx.class, "fore"); // porta 0 - expansion
        rightarmMotor = hardwareMap.get(DcMotorEx.class, "forearm"); // porta 1 - expansion

        leftarmMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightarmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftarmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightarmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftarmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightarmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftarmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightarmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armController = new PIDController(0.1, 0, 0.0015);
    }

   /**
     * Movimenta o braço do robô com a força escalonada.
     * Para alterar o fator, veja o {@link RobotConstants}
     * @param target do gamepad do jogador
     */

   public void PIDcontroller(double target) {
        double ticks_in_degrees = 280 / 360.0;
        double ZERO_OFFSET = 45;
       //adicionamos as variáveis do PID para nossos motores
       armController.setPID(0.1, 0, 0.0015);
       int leftArmPos = leftarmMotor.getCurrentPosition();
       int rightArmPos = rightarmMotor.getCurrentPosition();
       int armPos = (leftArmPos + rightArmPos) / 2;
       double pid = armController.calculate(armPos, target);
       double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * RobotConstants.FeedForward;

       double power = pid + ff;
       leftarmMotor.setPower(power);
       rightarmMotor.setPower(power);
   }

    public void moveArmsManually(double joystick) {

        double controlPower = joystick * RobotConstants.ARM_POWER_SCALE;
        rightarmMotor.setPower(controlPower);
        leftarmMotor.setPower(controlPower);
    }

    public void setArmPosition(ArmStage position) {
        switch (position) {
            case CLOSED:
                armTarget = RobotConstants.ARM_CLOSED_GOAL;
                break;
            case LOW:
                armTarget = RobotConstants.ARM_LOW_GOAL;
                break;
            case MEDIUM:
                armTarget = RobotConstants.ARM_MEDIUM_GOAL;
                break;
            case HIGH:
                armTarget = RobotConstants.ARM_HIGH_GOAL;
                break;
        }
        PIDcontroller(armTarget);
    }

    public void setVoltage(double voltage) {
        robotVoltage = voltage;
    }


    //
    public double getArmPID() {
        return armPID;
    }

    //
    public double getForearmFeedforward() {
        return armFeedforward;
    }

    //
    public double getForearmPID() {
        return armPID;
    }

}