package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    private final DcMotor intake;
    private final double INTAKE_POWER = 1;
    private final double OUTTAKE_POWER = 1;

    public Intake (OpMode opMode){
        intake =  opMode.hardwareMap.get(DcMotor.class, "Intake");
        init();
    }

    public void init(){
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void stop (){
        intake.setPower(0);
    }

    public void intake(){
        intake.setPower(INTAKE_POWER);
    }

    public void outtake(){
        intake.setPower(OUTTAKE_POWER);
    }
}

