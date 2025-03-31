## Hi it's me again :3
In this document I'll go over how the built in encoders on DCMotors work, and when you need to use them.

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
    ``<code>``
    }
}
``
To use encoders, you need to change modes which your motor runs on:
``motor.setMode(DcMotor.RunMode.``<insert run mode here>``)``

The possible modes are:

``RUN_TO_POSITION``
Used to tell the motors to run for a specific number of ticks.
Before setting it to this mode, you need to specify a target position, using:
``motor.setTargetPosition(``<number of ticks>``)``
This will run until the mode is changed, or a different target position is given.

``RUN_USING_ENCODER``
Encoders are used to automatically control the velocity, with a PID controller (?)

``RUN_WITHOUT_ENCODER``
This DOES NOT turn off the encoder! It simply means the build it velocity control is not used.

``STOP_AND_RESET_ENCODER``
This does what you think it does! It resets the encoders, effectively "zeroing" them. It is generally good practice to do this at the start of the OpMode.


## In TeleOp (driven) phase
It is very important to add limits to certain parts of your robot. 
They can serve several functions including:
* Reducing risk of damaging the robot
* Staying within the size limits, as stated by the rules
* Making sure things don't get stuck 
* and more....

Our robot had an arm which rotated along the vertical axis, and extended horizontally, so it needed limits on both mechanisms.

Now onto *how* to add those limits...

You need to start by working out what values you would like to limit the motion to. 
To measure these, you can set up telemetry [see my telemetry guide!!] to continuously print out the position of the motors using:
<motor>``.getCurrentPosition()``
Then move parts using the controller and record the values where it should be limited at. (Best to do this a few times and take the lower bounds)
Alternatively, you can physically move the robot (i.e. spin the wheels) and it will still count and output the counts! 

Then add if statements around setting the power, to ensure that the motor cannot be requested to move further than the limit. 

## In autonomous phase
In the autonomous phase, encoders can be use to get the robot to move a specific distance, or to a specific position.
Depending on your specific setup, you may need to measure, or calculate, the number of counts per metre, CM or inch, to easily give instructions.
What I have done is just estimate the correct count numbers through trial and error, as the turning function was rather unreliable, but tha tis definitely not an ideal or proper way of doing it.

An example of a few lines of autonomous code to move a motor forwards 1000 counts is:
``motor.setTargetPosition(1000);
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor.setPower(0.3);
while(motor.isBusy()){}``
It is very important to wait while the motor is busy, otherwise it will move onto the next instruction without reaching the target position.
I made a "CheckBusy()" function, which included telemetry within the loop, so that it is constantly updating.
