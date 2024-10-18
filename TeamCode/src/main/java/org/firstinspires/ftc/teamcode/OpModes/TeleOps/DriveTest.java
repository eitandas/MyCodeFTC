package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.Poses.Vector2d;

@TeleOp(name = "DriveTest", group = "TeleOps")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(this);

        while (opModeIsActive()) {
            drive.fieldCentricDrive(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);
        }
    }
}
