package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Vector;

@Autonomous
public class Main_Autonomous extends LinearOpMode {
    private int[][] AprilTagCoords =
    {
        {-72, 48},
        {0, 72},
        {72, 48},
        {72, -48},
        {0, -72},
        {-72, -48}

    };

    //objects
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor left_motor;
    private DcMotor right_motor;
    private Servo claw_servo;


    @Override
    public void runOpMode()
    {
        left_motor = hardwareMap.get(DcMotor.class, "left");
        right_motor = hardwareMap.get(DcMotor.class, "right");
        claw_servo = hardwareMap.get(Servo.class, "claw");
        //hardware map
        initAprilTag();
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "initialised");
        telemetry.update();
        waitForStart();

        while(opModeIsActive())
        {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if(currentDetections.size() != 0)
            {
                GetPosition(currentDetections.get(0));
            }

            telemetry.update();

        }
        visionPortal.close();
    }
    private void initAprilTag() //inches + radians
    {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        builder.addProcessor(aprilTag);

        visionPortal = builder.build();


    }

    public Vector<Double> GetPosition(AprilTagDetection tag)
    {
        double ang = GetRotation(tag);
        Vector<Double> coords  = new Vector<>();

        double tX = AprilTagCoords[tag.id -11][0];
        double tY = AprilTagCoords[tag.id -11][1];

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

    public void GoTo(Vector<Double> position, Vector<Double> currentPos, double angle)
    {



    }


}
