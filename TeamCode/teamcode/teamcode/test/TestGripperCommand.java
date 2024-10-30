package org.firstinspires.ftc.teamcode.test;

/**
 * Teste focado em utilizar o sistema de commandos
 * junto dos recursos do GamepadEx, providos pela FTCLib
 */
/*@TeleOp(name = "TestGripperCommand", group = "Test")
@Disabled
public class TestGripperCommand extends CommandOpMode {
    //private Servo servo;
    private GamepadEx player2;

    /*@Override
    public void initialize() {
        servo = new Servo(hardwareMap);
        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();


        // quando apertado uma vez, fecha a garra
        // na segunda, ele abre a garra
        player2.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(gripper::closeGrip),
                        new InstantCommand(gripper::openGrip),
                        true
                );

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new InstantCommand(() -> gripper.moveWrist(1)), true)
                .whenInactive(new InstantCommand(() -> gripper.moveWrist(0)), true);

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new InstantCommand(() -> gripper.moveWrist(- 1)), true)
                .whenInactive(new InstantCommand(() -> gripper.moveWrist(0)), true);
     }

    @Override
    public void run() {

        player2.readButtons();

        CommandScheduler.getInstance().run();

        telemetry.addData("gripper position", gripper.gripper.getCurrentPosition());
        telemetry.addData("wrist A power", gripper.wristServoA.getPower());
        telemetry.addData("wrist B power", gripper.wristServoB.getPower());
        telemetry.addData("is wristA enabled?: ", gripper.wristServoA.isPwmEnabled());
        telemetry.addData("is wristB enabled?: ", gripper.wristServoB.isPwmEnabled());
        telemetry.addData("is gripper enabled?: ", gripper.gripper.isMotorEnabled());
        telemetry.update();
    }*/
//}
