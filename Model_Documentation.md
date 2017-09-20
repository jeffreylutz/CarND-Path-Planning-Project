Path Planning Project Write-up

By:  Jeffrey Lutz

Background:
The goal of the project was to develop a trajectory generation and path planning.

Path Planning:
  The first task undertook was to use the existing waypoints to create
a spline model to follow.  Then, the calculation of distance every 20 ms. of time
was used to compute the car's X and Y positions.

The first challenge I was confronted with was to determine the maximum jerk.
I set the extreme case of turning the most aggressive corner along with changing
lanes with the same curvature of the lane at full speed.  This set the rate of lane
change (aka speed of changing lane)

The second issue was to maintain safe distance to the closest car in front and to the rear.
I added extra distance to the rear to allow for fast approaching cars that would
collide with.

I had to tweak the logic in order to push the car to the allowable safe distance without
falling behind over time.  It was a matter of setting the slow down rate to match the
response time of the controller/vehicle.  Essentially, there is a 50 step queue of path
that must be consumed before speed adjustments are made.

Then the next issue was how to determine which lane to switch to.  I realized that determining
the speed of lanes other than the current lane requires looking further ahead than
just the 1 x SAFE FRONT DISTANCE.  Other lanes look at 1.5 x SAFE FRONT DISTANCE and
current lane switched to 1.1 x SAFE FRONT DISTANCE.  This logic allows the car to decide on
switching lanes before being forced to slowdown with the 1.0 x SAFE FRONT DISTANCE
is made.

The next item I undertook was to figure out improvements to the logic so that safety
is maintained but performance is not compromised.  I realized that adding logic so that
the car would seek the middle lane even without car in front vehicle.  The value
of doing this is that once the car is in the middle lane an a front vehicle is present,
then the car will have options to switch to either left or right lane.  This improves
performance.

