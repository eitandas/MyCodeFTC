package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.Poses.Pose2d;

@Autonomous(name = "DriveAutoTest", group = "Autonomous")
public class DriveAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(this);

        if (opModeIsActive()) {
            drive.autonomusDrive(new Pose2d(1000, 2000, 30), 2000);
        }
    }
}
