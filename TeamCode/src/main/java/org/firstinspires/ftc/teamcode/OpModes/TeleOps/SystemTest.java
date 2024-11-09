package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.Utils.Poses.Vector2d;

@TeleOp(name = "SystemTest", group = "TeleOp")
public class SystemTest extends LinearOpMode {

    private int liftLevel = 0;
    private int angleLevel = 0;
    private int extendLevel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(this);
        Intake intake = new Intake(this);
        Lift lift = new Lift(this);
        Arm arm = new Arm(this);
        while (opModeIsActive()) {

            drive.fieldCentricDrive(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);
            lift.liftByLevels(liftLevel);
            arm.setArmExtend(extendLevel);
            arm.setArmAngle(angleLevel);

            if (gamepad2.dpad_up) {
                liftLevel = 2;
            } else if (gamepad2.dpad_right) {
                liftLevel = 1;
            } else if (gamepad2.dpad_down) {
                liftLevel = 0;
            }

            if (gamepad2.cross){
                angleLevel = 0;
            }
            else if (gamepad2.circle){
                angleLevel = 1;
            } else if (gamepad2.triangle) {
                angleLevel = 2;
            }

            if (gamepad2.dpad_left) {
                extendLevel = 3;
            } else if (gamepad2.square) {
                extendLevel = 0;
            }

            if (gamepad2.right_bumper) {
                intake.intake();
            } else if (gamepad2.left_bumper) {
                intake.outtake();
            } else {
                intake.stop();
            }


        }
    }
}
