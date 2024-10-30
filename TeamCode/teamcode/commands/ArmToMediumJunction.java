package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmToMediumJunction extends SequentialCommandGroup {
    public ArmToMediumJunction(DestemidosBot robot) {
        super(
                //new InstantCommand(()->robot.armSystem.setArmPosition(ArmSystem.ArmStage.MEDIUM)),
                //new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.MEDIUM))
                //new InstantCommand(()->robot.simpleArm.goToPosition(ArmSystem.ArmStage.MEDIUM))
                new InstantCommand(()->robot.simpleArm.goToPosition2(RobotConstants.ARM_MEDIUM_GOAL))
        );
    }
}
