package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.PIDFController;

public class Lift {
    private final DcMotorEx lift;
    private final PIDFController pidf = new PIDFController(0, 0, 0, 0);
    private final double[] levels = {0, 1000, 2000, 3000};
    private final double tolerance = 0;
    private double position;

    public Lift(OpMode opMode) {
        lift = opMode.hardwareMap.get(DcMotorEx.class, "Lift");
    }

    public void init(){
        resetEncoders();
        setTolerance();
        update();
    }

    public void setTolerance(){
        pidf.setTolerance(tolerance);
    }

    public void resetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        position = lift.getCurrentPosition();
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public void liftByLevels(int level) {
        double sp = levels[level];
        pidf.setSetPoint(sp);
        while (!pidf.atSetPoint()) {
            setPower(pidf.calculate(position, sp));
        }
    }
}
