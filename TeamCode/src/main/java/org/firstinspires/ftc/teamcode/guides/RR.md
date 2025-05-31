http://192.168.43.1:8080/dash

``ForwardPushTest``
Goal: determine inPerTick empirically

Place the robot on the tiles with plenty of room in front. Square the robot up with the grid and make note of its starting position. Run ForwardPushTest and then slowly push the robot straight until the end of the tiles. Record the “ticks traveled” from telemetry before stopping the op mode. Without moving the robot, record also the forward distance traveled on the tiles in inches. Set the inPerTick variable in your drive class to the real distance traveled divided by the ticks traveled.

Make sure the wheels don’t slip! The motors should spin freely. If the bot is too light or otherwise slipping, you can use theoretical values for ticks per revolution, gear ratio, and wheel diameter to compute inPerTick instead.
 
= 0 -> something needs reversing

``AngularRampLogger``
This routine is very similar to the last except that it rotates in place instead of moving forward. As before, you can change the power ramping parameters inside the power() method. Run the routine using the same instructions.

Drive Encoders 
Goal: determine trackWidthTicks, kS, and kV empirically

Go to http://192.168.43.1:8080/tuning/drive-encoder-angular-ramp.html and click the “Latest” button. Use the instructions from the ForwardRampLogger analysis to get kS, kV from the the “Ramp Regression” plot. Then use the same outlier-exclusion technique on the “Track Width Regression” and set trackWidthTicks to the “track width” value when you’re finished.

``ManualFeedforwardTuner``
Goal: Fine-tune kS, kV and add in kA

This routine repeatedly drives forward and back DISTANCE units, giving you an opportunity to finalize the feedforward parameters.
Open FTC Dashboard by navigating to http://192.168.43.1:8080/dash. Run the op mode, and graph vref against v0. Set kA to a small value like 0.0000001 and slowly increase it by a factor of ten until it starts effecting the plot. Try to make the two lines in chart as close together as possible.
At this point, the robot still has no feedback and may drift over many cycles. Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing you to reset the position of the bot. Pressing B/O (Xbox/PS4) will return to the tuning process.


``ManualFeedbackTuner``
Goal: Tune the feedback parameters
This routine also goes back and forth DISTANCE units but using combined feedforward and feedback.

Use this opportunity to tune the feedforward parameters of your trajectory following controller.
TankDrive: The two Ramsete gains shouldn’t need much tuning. According to the FRC Docs, “Larger values of bBar make convergence more aggressive like a proportional term whereas larger values of zeta provide more damping in the response.”

``SplineTest``
