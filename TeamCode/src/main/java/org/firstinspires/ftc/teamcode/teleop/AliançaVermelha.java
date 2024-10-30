package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.groupcommands.ForwardArmToBackdrop;
import org.firstinspires.ftc.teamcode.commands.simplecommands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.simplecommands.FistCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@TeleOp
public class AliançaVermelha extends CommandOpMode {

    private DestemidosBot robot;
    private GamepadEx player1;
    private GamepadEx player2;



    @Override
    public void initialize() {
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();


        player1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> robot.servoSystem.launchDrone()));

        player1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(new InstantCommand(() -> robot.servoSystem.moveHook(0.4)))
                .whenInactive(new InstantCommand(() -> robot.servoSystem.moveHook(0)));

        player1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new InstantCommand(() -> robot.servoSystem.moveHook(-0.4)))
                .whenInactive(new InstantCommand(() -> robot.servoSystem.moveHook(0)));



        player2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new InstantCommand(() -> robot.servoSystem.moveFist(1)))
                .whenInactive(new InstantCommand(() -> robot.servoSystem.moveFist(0)));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(new InstantCommand(() -> robot.servoSystem.moveFist(-1)))
                .whenInactive(new InstantCommand(() -> robot.servoSystem.moveFist(0)));

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> robot.servoSystem.closeWrist()));
 
        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> robot.servoSystem.openWrist()));

        player2.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(new InstantCommand(() -> robot.simpleArm.suspendBot(1)))
                .whenInactive(new InstantCommand(() -> robot.simpleArm.clayMotor.setPower(0)));

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenActive(new InstantCommand(() -> robot.simpleArm.suspendBot(-1)))
                .whenInactive(new InstantCommand(() -> robot.simpleArm.clayMotor.setPower(0)));

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new ArmCommand(robot, 50 ),
                        new InstantCommand(() -> robot.simpleArm.unlockArm()),
                        true
                        );

        /* player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmToHighJunction(robot));*/

    }


        @Override
        public void run() {
            CommandScheduler.getInstance().run();
            player2.readButtons();
            player1.readButtons();

        /*if(player2.gamepad.right_trigger > 0.0) {
            robot.gripper.gripper.setPower(player2.gamepad.right_trigger * 0.60);
        }

        if(player2.gamepad.left_trigger > 0.0) {
            robot.gripper.gripper.setPower(-player2.gamepad.left_trigger * 0.60);
        }*/

            robot.drivetrain.standardMecanumController(gamepad1);
            robot.simpleArm.forceArm(-player2.gamepad.right_stick_y);

        }


    }

