package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

/*
 * TestSensorIMU - OpMode dedicado a leitura de informações
 * detalhadas como: Posição, Aceleração, Velocidade Angular, Ori9entação
 * e etc.. todas providas pelo sensor embutido no próprio controlhub
 * 
 * para outros exemplos: https://stemrobotics.cs.pdx.edu/node/7265.html 
 */

@TeleOp(name = "TestSensorIMU", group = "Test")
public class TestSensorIMU extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DestemidosBot robot = new DestemidosBot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            robot.drivetrain.standardMecanumController(gamepad1);

            // pegando informações do imu
            AngularVelocity robotAngVel = robot.localizationSystem.getAngularVelocity(AngleUnit.DEGREES);

            YawPitchRollAngles robotAngles = robot.localizationSystem.getSensorIMU().getRobotYawPitchRollAngles();

            telemetry.addData("Sensor IMU - Velocidade Angular em Graus : ", "X: %f / Y: %f / Z: %f",
                    robotAngVel.xRotationRate,
                    robotAngVel.yRotationRate,
                    robotAngVel.zRotationRate
            );

            telemetry.addData("Sensor IMU - Ângulo em Graus: ", "Yaw: %f / Pitch: %f / Row: %f",
                    robotAngles.getYaw(AngleUnit.DEGREES),
                    robotAngles.getPitch(AngleUnit.DEGREES),
                    robotAngles.getRoll(AngleUnit.DEGREES)
            );
            telemetry.update();
        }
    }
}
