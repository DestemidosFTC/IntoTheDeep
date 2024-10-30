package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleArm;

/**
 * OpMode focado em experimentação de novas idéias
 * e de medição das informações do robô
 */
@TeleOp(name = "TESTBOT", group = "Test")
@Disabled
public class FTC_TESTBOT extends CommandOpMode {
    private DestemidosBot robot;
    private GamepadEx player2;
    private SimpleArm simpleArm;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new DestemidosBot(hardwareMap);
        simpleArm = new SimpleArm(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();
        register(robot.servo, simpleArm);

        /*
        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(-1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));
        */

        player2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(
                        () -> simpleArm.goToPosition(RobotConstants.ARM_CLOSED_GOAL))
                );

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(
                        () -> simpleArm.goToPosition(RobotConstants.ARM_LOW_GOAL))
                );

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(
                        () -> simpleArm.goToPosition(RobotConstants.ARM_MEDIUM_GOAL))
                );

        player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(
                        () -> simpleArm.goToPosition(RobotConstants.ARM_HIGH_GOAL))
                );

    }

    @Override
    public void run() {
        double antes = System.currentTimeMillis();

        double energy = robot.voltageSensor.getVoltage();
        player2.readButtons();
        CommandScheduler.getInstance().run();
        robot.drivetrain.standardMecanumController(gamepad1);

        double depois = System.currentTimeMillis();

        /*
        if(player2.gamepad.right_trigger > 0.0) {
            robot.gripper.gripper.setPower(player2.gamepad.right_trigger * 0.60);
        }

        if(player2.gamepad.left_trigger > 0.0) {
            robot.gripper.gripper.setPower(-player2.gamepad.left_trigger * 0.60);
        }
         */

        telemetry.addData("SIMPLE ARM: left motor position: ", simpleArm.leftMotor.getCurrentPosition());
        telemetry.addData("SIMPLE ARM: right motor position: ", simpleArm.rightMotor.getCurrentPosition());
        telemetry.addData("SIMPLE ARM: target: ", simpleArm.pidPower);
        telemetry.addData("SIMPLE ARM: pidPower: ", simpleArm.target);
        telemetry.addData("SIMPLE ARM: arm output: ", simpleArm.armCommand);

        telemetry.addData("voltagem do controlhub", energy);
        telemetry.addData("tempo de loop (ms)", depois - antes);
        telemetry.update();
    }
}