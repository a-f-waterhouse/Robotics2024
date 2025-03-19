## Hi it's me again :3
In this document I'll go over how encoders on DCMotors work, and when you need to use them.

## Basics

To start with: make sure the encoder cables are plugged in!! and make sure they are plugged into the correct ports!!

For the purpose of this guide, we assume a motor (creatively named ``motor``) has been all set up, so that this is our code inside the OpMode class looks a bit like this:
``
private DcMotor motor;
@Override
public void runOpMode()
{
    motor = hardwareMap.get(DcMotor.class, "motor");
    while(opModeIsActive())
    {
    //code
    }
}
``
To use encoders, you need to change modes which your motor runs on 
``motor.setMode(DcMotor.RunMode.``insert run mode here!!``)``

The possible modes are: //NOTE TO SELF: EXPLAIN EACH ONE
``RUN_TO_POSITION``
Used to tell the motors to run for a specific number of ticks.
Before setting it to this mode, you need to specify a target position, using
``motor.setTargetPosition(``number of ticks``)`` 


``RUN_USING_ENCODER``


``RUN_WITHOUT_ENCODER``


``STOP_AND_RESET_ENCODER``


## In TeleOp (driven) phase
It is very important to add limits to certain parts of your robot. 
They can serve several functions including:
* Reducing risk of damaging the robot
* Staying within the size limits, as stated by the rules
* Making sure things don't get stuck 
* and more....

Our robot had an arm which rotated along the vertical axis, and extended horizontally, so it needed limits on both mechanisms.

Now onto *how* to add those limits...



## In autonomous phase





## THIS WORKS!!!!!!

``left_drive.setTargetPosition(1000);
right_drive.setTargetPosition(1000);
left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
left_drive.setPower(0.3);
right_drive.setPower(0.3);``

