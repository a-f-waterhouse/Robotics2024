192.168.43.1:8080/dash

# Feedforward Tuning
``ManualFeedForwardTuner``
Goal: poseVelocity roughly = targetVelocity, thru tuning DriveConstants: kA, kV and kStatic
Process:
* kV = 1/MAX_VEL
* increase kV until touching top of targetVelocity plateau
* increase kA to try to get slopes to match
* YIPPEE!!
* copy numbers into ``DriveConstants``
![deetz-tuning-half.da5fb022.jpg](..%2F..%2F..%2F..%2F..%2F..%2F..%2F..%2F..%2FAppData%2FLocal%2FTemp%2Fdeetz-tuning-half.da5fb022.jpg)

-> Overshooting 10% is normal.

# Straight
``StraightTest``
Goal: check good so far
Process:
* Measure dist travelled: should = 60inches
* Run a few times to check consistency
* YIPPEE!!

-> drifting a little is ok

Goal: localization
Process:
* Compare measurements with finalX
* if !=, do GEAR_RATION * (measured dist/reported value)
* Do a couple times to fine tune

# Track Width Tuning
``MaxAngularVelocityTuner``
Process:
* MAX_ANG_VEL = value outputted

``TrackWidthTuner``
Goal: Should turn 180 5 times
Process:
* TRACK_WIDTH = outputted value
* If unreasonable, can tune manually:
* <180 increase, >180 increase
* within a few degrees accuracy required

# Turn Test
``TurnTest``
Goal: check not wayyyy off
Process:
* Should turn 90
* If not, change to 180 and if that works then all good apparently!!

-> Should turn anti-clockwise, otherwise swap motor config idk

# Localisation Test
``LocalisationTest``
Process:
* Enable field view on dashboard
* Drive around - green = target, blue = actual
* If position/orientation all good then YIPPEE!!

# Follower PID 
-> NOTE: taken from official docs, not LearnRR because they hate Tank Drive :(
``BackAndForth`` and ``FollowerPIDTuner``
Goal: Tune other PIDs [in SampleTankDrive]
Process:
* Field view
????????????


# Spline Test
Goal: double check everything is awesome :)