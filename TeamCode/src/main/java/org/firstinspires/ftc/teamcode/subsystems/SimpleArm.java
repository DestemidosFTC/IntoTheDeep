package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class SimpleArm implements Subsystem {

    // Motores
    public final DcMotorEx leftMotor;
    public final DcMotorEx rightMotor;
    public final DcMotorEx clayMotor;

    // Controlador
    private final PIDController controller;

    // Medidas do sistema
    public int target;
    public double pidPower;
    public double armCommand;
    public ElapsedTime PIDtimer;

    // Construtor padrão pra o mapeamento e iniciazlização dos componentes
    public SimpleArm(HardwareMap hardwareMap) {

        // mapeia os motores de acordo com os nomes no drivestatiom
      
        // TODO: defina a ID de cada motor

        leftMotor = hardwareMap.get(DcMotorEx.class, "fore"); //porta 0, expansion
        rightMotor = hardwareMap.get(DcMotorEx.class, "forearm"); //porta 1, expansion
        clayMotor = hardwareMap.get(DcMotorEx.class, "bombs"); //porta 3, expansion

        // reseta os encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // define a direção dos motores
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // desativa o PID interno do motor, que atualiza numa frequência mais lenta que o código
        // por isso, nós queremos ter o controle da "força pura" do motor
        // referência: https://www.ctrlaltftc.com/practical-examples/ftc-motor-control
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // define como o motor deve reagir quando chegar no "zero" de energia
        // nesse caso, queremos que ele desacelere imediatamente
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // inicializa o controlador PID com as váriavies editáveis do ftc-dashboard
        // usamos as váriáveis do RobotConstants.ARM_POSITION_PID
        controller = new PIDController( 0.2, 0, 0.0015 );

        // zeramos o alvo, pra evitar riscos
        target = 0;

        //timer
        PIDtimer = new ElapsedTime();
        PIDtimer.reset();
    }

    // Essa função fica sempre loopando durante a execução do robô
    // ela vai sempre checar pela posição do motor, e mandar pro controlador
    @Override
    public void periodic() {
        // durante o loop, mandamos as infromações pros motores e executamos
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(armCommand);
        rightMotor.setPower(armCommand);
    }

    public void goToPosition(int goal) {
        controller.setPID(0.4, 0, 0.0015);
        target = goal;

        // pegamos a posição dos motores
        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

        // vamos fazer uma média das posições, mas como a posição do motor é sempre um valor int
        // iremos "arredondar pra baixo", pra compensar uma possível diferença entre as medidas,
        // usando a função floor()
        double positionAverage = (leftPos + rightPos) / 2.0;
        int currentPosition = (int) Math.floor(positionAverage);

        // aplicamos os valores no pid, e calculamos a força necessária
        pidPower = controller.calculate(currentPosition, target);

        // limitamos a força do pid em um intervalo controlado
        // e salvamos em um comando
        armCommand = Range.clip( pidPower,
                -RobotConstants.ARM_PID_MIN_POWER_LIMIT,
                RobotConstants.ARM_PID_MAX_POWER_LIMIT);

        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(armCommand);
        rightMotor.setPower(armCommand);
    }

    public void setPosition(int target) {
        double ticks_in_degrees = 360/288.0;
        PIDtimer.reset();
        double FeedFoward = Math.cos(Math.toRadians(target / ticks_in_degrees)) * RobotConstants.FeedForward;
        while (PIDtimer.seconds() > 3.0) {
            int leftarmPosition = leftMotor.getCurrentPosition();
            int rightarmPosition = rightMotor.getCurrentPosition();
            double armPosition = (leftarmPosition + rightarmPosition) / 2.0;
            int currentPosition = (int) Math.floor(armPosition);
            double PID = controller.calculate(currentPosition, target);

            double power = PID + FeedFoward;
            leftMotor.setPower(-power);
            rightMotor.setPower(-power); }
        leftMotor.setPower(FeedFoward);
        rightMotor.setPower(FeedFoward);
        PIDtimer.reset();
    }

    public void forceArm(double power) {
        double scale = power * RobotConstants.ARM_POWER_SCALE;
        leftMotor.setPower(scale);
        rightMotor.setPower(scale);
    }

    public void suspendBot(double power) {
        clayMotor.setPower(power);
    }


    public void unlockArm() {
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}


