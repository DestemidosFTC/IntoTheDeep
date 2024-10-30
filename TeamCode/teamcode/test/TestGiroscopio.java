package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.controllers.GyroController;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@Autonomous(name = "Test Giroscopio", group = "Test")
@Disabled
public class TestGiroscopio extends LinearOpMode {
    public static double targetAngle = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        DestemidosBot robot = new DestemidosBot(hardwareMap);
        GyroController gyroController = new GyroController(2.03);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // resetando o angulo
        robot.localizationSystem.resetAngle();

        waitForStart();
        while(opModeIsActive()) {

            // giro de 90 graus por padrão
            double robotAngle = robot.localizationSystem.getRobotHeading();

            double output = gyroController.calculate(Math.toRadians(targetAngle), robotAngle);
            double motorOutput = Range.clip(output, -1.0,1.0);

            robot.drivetrain.setMotorsPower(-motorOutput, -motorOutput, motorOutput, motorOutput);

            telemetry.addData("target angle", targetAngle);
            telemetry.addData("robot angle", Math.toDegrees(robotAngle) );
            telemetry.addData("pid", output);
            telemetry.update();
        }
    }
}
