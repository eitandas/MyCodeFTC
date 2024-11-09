package org.firstinspires.ftc.teamcode.OpModes.ThreadTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.Utils.Poses.Pose2d;

@Autonomous
public class Test extends LinearOpMode {
    private DriveTrain driveTrain;
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        lift = new Lift(this);
        waitForStart();
        Thread encoderThread = new Thread(() -> {
            try {
                while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                    driveTrain.updateEncoders();
                    lift.updateEncodoers();
                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        });
        Thread liftThread = new Thread(() -> {
            try {
                while (!Thread.currentThread().isInterrupted() && opModeIsActive() && !lift.atSetPoint()) {
                    lift.liftByLevels();
                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        });
        lift.setLevel(1);
        encoderThread.start();
        liftThread.start();
        if (opModeIsActive()) {
            driveTrain.autonomusDrive(new Pose2d(30, 30, 0), 5);
            lift.setLevel(2);
        }
        liftThread.interrupt();
        encoderThread.interrupt();
        lift.stop();
        driveTrain.stop();
    }
}
