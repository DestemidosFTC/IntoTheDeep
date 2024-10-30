package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@TeleOp
public class FTC_TESTROBOT extends CommandOpMode {

    private DestemidosBot robot;
    private GamepadEx player2;

    @Override
    public void initialize() {
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();


        player2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> robot.simpleArm.forceArm(-1)));


    }


        @Override
        public void run() {
            CommandScheduler.getInstance().run();
            player2.readButtons();


        }


    }

