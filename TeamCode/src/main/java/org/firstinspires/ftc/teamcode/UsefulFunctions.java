package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Vector;

@Autonomous
public class UsefulFunctions extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        waitForStart();
        while (opModeIsActive() ) {

            //telemetry.addLine(String.format("position: %03d ", m.getCurrentPosition()));
            telemetry.update();
        }

    }
    public Vector<Double> GetPosition(AprilTagDetection tag)
    {
        double ang = GetRotation(tag);
        Vector<Double> coords  = new Vector<>();

        double tX = 0;//AprilTagCoords[tag.id -11][0];
        double tY = 0;//AprilTagCoords[tag.id -11][1];

        double x, y;

        if(tag.id == 12 || tag.id == 15) //front + back walls
        {
            x = tX - tag.ftcPose.range * (Math.cos(Math.PI - tag.ftcPose.bearing + ang));
            y = tY - tag.ftcPose.range * (Math.sin(Math.PI - tag.ftcPose.bearing + ang));
        }
        else
        {
            x = tX + tag.ftcPose.range * (Math.cos(tag.ftcPose.bearing - ang));
            y = tY - tag.ftcPose.range * (Math.sin(tag.ftcPose.bearing - ang));
        }

        coords.add(x);
        coords.add(y);
        telemetry.addLine(String.format("x: %6.1f    y: %6.1f", x, y));


        return coords;
    }

    public double GetRotation(AprilTagDetection tag)
    {
        double angle = tag.ftcPose.yaw;
        if(tag.id == 11 || tag.id == 16) //left wall
        {
            angle-= (Math.PI);
        }
        else if(tag.id == 12) //back wall
        {
            angle-= (Math.PI/2);
        }
        else if(tag.id == 13 || tag.id == 14) //right wall
        {

        }
        else if(tag.id == 15) //front wall
        {
            angle+=(Math.PI/2);
        }

        return (angle + 3 * Math.PI) %(2*Math.PI) - Math.PI;
    }



}
