/*package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@TeleOp
public class AliançaAzul extends CommandOpMode {
    private DestemidosBot robot;
    private GamepadEx player2;

    @Override
    public void initialize() {
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);
        CommandScheduler.getInstance().reset();

        schedule();
        register(robot.servoSystem, robot.armSystem);

        player2.getGamepadButton( GamepadKeys.Button.DPAD_UP)
                .whenActive(new InstantCommand(() -> robot.servoSystem.moveFist(1)))
                .whenInactive(new InstantCommand(() -> robot.servoSystem.moveFist(0)));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new InstantCommand(() -> robot.servoSystem.moveFist(-1)))
                .whenInactive(new InstantCommand(() -> robot.servoSystem.moveFist(0)));

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(robot.servoSystem::closeWrist),
                        new InstantCommand(robot.servoSystem::openWrist),
                        true
                );

    }


    @Override
    public void run() {
        double energy = robot.voltageSensor.getVoltage();

        CommandScheduler.getInstance().run();

        player2.readButtons();

        robot.armSystem.setVoltage(energy);

        robot.drivetrain.updateVoltage(energy);
        robot.drivetrain.standardMecanumController(gamepad1);
    }
}
*/