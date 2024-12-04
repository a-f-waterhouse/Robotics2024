package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class UsefulFunctions extends LinearOpMode {

    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    private Servo test_servo  = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor m;
    int[] positions = {0,-45};
    int clawPos = 0;


    @Override
    public void runOpMode()
    {
        test_servo = hardwareMap.get(Servo.class, "test_servo");
        /*m = hardwareMap.get(DcMotor.class,"test_motor" );

        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);;
        waitForStart();
        m.setTargetPosition(288);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        waitForStart();
        while (opModeIsActive() ) { //&& (m.isBusy()
            if(gamepad1.x)
            {
                clawPos = (clawPos+1)%2;
                //test_servo.setPosition(DegreesToPos(positions[clawPos]));
                test_servo.setPosition(0);
                telemetry.addLine(String.format("position: %03d ", clawPos));
                //m.setPower(0.6);
            }
            //telemetry.addLine(String.format("position: %03d ", m.getCurrentPosition()));
            telemetry.update();
        }

    }



}
