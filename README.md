# Udacity Capstone Project 

This is our `SDC Fun` team repo for the capstone project in Udacity's Self-Driving Car Nanodegree 
program.  Our team members are:

 * James Barfield (jamesbarfield87@gmail.com)
 * Nick Condo(nicholas.condo@gmail.com)
 * Jim Duan (jd@vehicular.ai)
 * Nimish Sanghi (nimish.sanghi@gmail.com)
 * Colin Shaw (colin.shaw@aya.yale.edu)



### Original sources

The original project repo can be found [here](https://github.com/udacity/CarND-Capstone), 
which has a lot of information about environment, simulator, etc.



### Building and running the project

This project depends on having CUDA and cuDNN installed in order to function
properly.  Reason is the inference speed for traffic light detection.  You will
notice a `requirements.txt` file in the root of the project.  If your machine does
not satisfy the above requirement, you will need to change the `tensorflow-gpu`
requirement to `tensorflow` to support CPU alone.  All other (python) dependencies
can be satisfied from the root of the project by `pip install -r requirements.txt`.  

Next go into the `/ros` directory and run `catkin_make`.  Be sure to source the 
project by running `. devel/setup.sh`.  At this point, if your environment is set 
up properly, you should be able to launch ROS with `roslaunch launch/styx.launch`.



### Traffic light detection

Traffic light detection required several steps to accomplish, the preliminary parts
being visible in the `/data_science/` directory, the latter parts in the main
project.  The first aspect that was addressed was collecting training data.  The full 
images were classified by hand after loading the `ROSBAG` and were output as `h5` for
simpler later processing.  The traffic lights were extracted from these collection 
of frames by color selection and bounded by using statistics based on k-means 
clustering of the resulting regions.  This provided information for training the 
network with labeled images as well as the specific regions where the lights existed
being annotated as well.

The available data was split into training, test and valiation sets.  Pre-trained 
VGG16 was used as a starting point with the classifier being re-trained for our 
specific problem.  The full images were what ended up being used for training, 
with most images in the training set containing no light, followed by examples
of red and green lights, and the fewest training examples being yellow lights.  The
training data was a combination of simulator images and provided Udacity track
images so that we would have the sense that the performace we saw in the simulator
would generalize to the real track since the training data contained both sets.  The
total number of training features used was 2703.  Augmention was used and included
the following transformations:

  * Rotation
  * Width shift
  * Height shift
  * Shear
  * Zoom
  * Horizontal image flip

The new classifier applied to VGG16 is a fully connected network with 256 neurons
connected to 4 output neurons with 50% dropout and softmax output.  This was trained 
using the `RMSprop` optimizer with categorical cross-entropy loss for 30 epochs using 
a batch size of 32.  The training and validation accuracies and losses were evaluated 
to ensure the model was improving but not overfitting.  Ultimately, the accuracy was 
about 99% on the validation set.  The model was fine tuned from this point by allowing
the last three convolutional layers to be trainable and repeating the training.  This
improve the validation accuracy to about 99.5%.  The model then evaluated random 
image samples to be sure it performed as expected.

The traffic light detector employs this model, classifying images that are exposed 
by the `/vehicle/traffic_lights`.  First the image is classified and the predicted
state is determined by taking the `argmax` of the four resulting classification
vectors.  Inference speed is about 15ms per frame using a GTX 1060.  If there is 
evidence of a traffic light in the frame, the nearest traffic light waypoint 
in front of the car is used as the waypoint.  An alternative to this 
mechanism of identifying the waypoint is to use information about
the car's pose and the camera to infer a point where the traffic light is using
projection to the plane the car is on.  This can then be used to identify the nearest
waypoint without `a priori` knowledge of the traffic light waypoints.  Both the
traffic light state and the waypoint of the traffic light are then passed to the
waypoint updated to apply logic to control the acceleration and braking of the 
car, though only if the traffic light detected has been experienced 
`STATE_COUNT_THRESHOLD` times in a row, which prevents false positives.



### Motion control

Motion control depends on knowledge of the waypoints.  In this case, the waypoints
are sufficiently close together to not require interpolation.  We first changed
`waypoint_loader.py` to publish a single, latched read of the waypoints, as 
it had initially been set to publish remarkably frequently and wasted a lot of
valuable resources.  This was done with latching a single `publish()` because 
the use of the topic is essentially to avail a static data set to various objects
that use the data as a property.  Doing this simply improved performance, though
this was fixed later in the Udacity repo.

The `waypoint_updater.py` code is critical to the project as it performs two main 
tasks.  First, it takes the current pose and other runtime properties, as well
as the static waypoints, and produces a `Lane` topic that consists of a subset of 
the waypoints starting with the first waypoint in front of the car, with a twist 
linear velocity set in the axis of travel.  Second, the traffic light control 
is performed, which is the topic of the next section.

We first identify which waypoint index is nearest in front of the car, as well as 
ensuring the heading of the waypoint while determining the nearest.  Next, 
depending on the control from the traffic light, we generate a velocity profile 
for waypoints in front of the car.  The profile that is used is simply a constant
of the desired car speed when the car should be driving or zero when the car should
be coming to a halt.  This constant list is integrated into a `Lane` topic 
and published.  We added a new topic that is intended to be invoked from the 
console using `rostopic pub ...` so that the speed setpoint can be adjusted at whim,
though there is a default representing the expected speed of the physical car.  

The waypoint follower implements what is known as the `pure pursuit` algorithm
(see [here](docs/Coulter.pdf) for more on that), which returns a `TwistStamped`
topic that is the computed best move for the car based on the algorithm.  We noted
that the implementation of the algorithm indexes the waypoints proportional to 
the speed of travel, so increased the `LOOKAHEAD_WPS` constant in `waypoint_updater.py`
to facilitate higher speed travel.  While we are aware the speeds that the
real car will travel are not as high, we wanted to be able to tune our simulated
car for reasonable travel at highway speeds.

`dbw_node.py` subscribes to the `TwistStamped` topic produced by the waypoint 
follower.  Here we seek to control the throttle, brake and steering of the 
car using the desired linear and twist velocities and the current velocity.  The 
main loop in `dbw_node.py` simply creates a convenient data structure that is
passed to an instance of our `Controller` class to achieve the desired control.

With respect to the throttle and brake, we implemented a single PID 
controller that has a split output for the throttle and brake.  This conditionally 
allows scaling for the throttle in the [0,1] range while allowing scaling for 
the brake deadband through maximum torque as defined by the car properties for
the braking.  The PID controller was tuned manually (see [here](docs/BasilioMatos.pdf) 
and [here](docs/ZieglerNichols.pdf), and more specifically 
[here](https://www.youtube.com/watch?v=drYO60z6_h4) for more on tuning). The 
scaling function for both acceleration and braking is a variant of 
soft-step.  The reasons for this are smooth transition (though the derivative is not
continuous at zero, this hardly matters since that is the point where the two
types of car control change), automatic bounding due to the asymptotic nature
of the function, and the ability to easily change the degree of control that 
will saturate the output.  This last property is helpful in both the case of 
accelerating and braking, as for large changes in target speed it makes it 
simple to engage the car to the extreme defined by the car properties and
project requirements.  Per the requirements, the control is responsive to the 
`dbw_enable` topic and resets the integral accumulator when this is toggled 
affirmatively.  It was also noticed that the integral accumulator tends to cause
starting problems when the car has stopped previously, so it is also reset when
the speed falls below a small value.

The steering control is more complex.  We compute a desired steering and current 
steering estimate using an instance of the `YawController` class with our 
desired and current steering information from the waypoint follower and current 
pose.  The `YawController` class was rewritten with this in mind.  The reason for
this is that since we are using a desired and current estimate so that we can control
the motion with a PID controller, we are not using all of the features of the 
existing controller, and it was possible to simplify it for our requirements.  The
reason for this approach is that in using only the original YawController, which
works very well for a constant speed, there is marked deviation in the steering as
the car comes to a halt.  This is because of the use in the controller of the ratio
of the desired and current speed, which affects the steering angle reported.  Using
the two simpler versions of the yaw controller and a PID controller to control the
*error* in steering yields better practical results. We pre-filter the steering 
error using a low pass filter, and then submit this to a PID controller.  Like the 
throttle and brake controller, this PID controller was also manually tuned, though more
effort was spent on it since the effects are more noticable. The PID controller is 
followed by a second low pass filter.  The filter parameters were selected to have
enough effect to smooth the course of the car, but not so much effect as to cause
excessive deviation at the sharpest turns.  



### Traffic light and motion control integration

There are a number of ways of accomplishing the integration between the traffic
light identification and the motion control system, some complicated and some
not.  The way that we selected was not as complicated.  However, there are a 
few things to note about the dynamics of the car that are important to the 
argument for why we chose the method we did.  

First, we know that the simulator is emulating the dynamics of the real car
by having mass as well as an inertial aspect to acceleration and braking.  Since
this is the case, we can control the car naturally with a binary control while
letting the PID controller provide the impetus for the desired motion.  We looked
into the idea of creating minimum jerk velocity profiles for future waypoints,
but the end result of this is simply increasing the stopping distance on a car
that already has the motion dynamics.

Second, one may consider planning a future waypoint where the stopping action 
should begin.  This is natural since we are projecting many waypoints into the
future and would like to stay ahead of what we need to execute.  The problem
with this is there is not an advantage over the simpler model of only looking 
at waypoints within the stopping distance of the car.  This can be trivially
determined from the speed of the car speed and our knowledge of the dynamics
of the brake torque and other properties of the car.  This is complicated slightly
by the fact that we have a smoothing function on the braking, but the way this is
implemented is to accommodate full braking with minimal braking control.  The 
reason for this is to allow for the smooth near-zero braking while at the same time
applying maximum braking based on the car properties when we need to come to a 
stop. 

With these ideas as our guides, we took the simplest approach first at integrating
the traffic light identification with the motion controller.  The traffic light
identification aspect of the project informs the waypoint updater of the next
traffic light waypoint and its associated color.  Simple logic based on the 
ideas mentioned above dictate how the car will react to it, either causing braking 
to zero speed or causing acceleration to a preset cruising speed.



### How we managed work in our team

One important aspect of working on a team to complete this project was
defining a good schedule with milestones for completion.  We wanted to
have an MVP ready to turn in by September 18, two full weeks prior to 
the project deadline, so that if there were any problems, unmet
requirements or new ideas there would be plenty of time to accommodate
so that we would be able to have our code run on the actual car.  When
we intitially formed the team we decided this accelerated schedule was 
desirable since we all wanted to achieve this shared goal.

In terms of work division, this project has two main components, training 
a model to identify the waypoint associated with the nearest red light 
in the direction of travel of the car, and waypoint management and motion 
control.  These tasks can be divided into a sequence of two asynchronous 
tasks followed by integration and testing. 

The members of our team selected what aspect to work on initially based
on personal preference, knowing that as the project evolved we would all
be involved in a wider view of it.  James and Nick predominantly worked 
on the model for the traffic light detector while Colin and Nimish 
predominantly worked on the motion control.  By the weekend of September 9
the whole team started working on integration to meet our MVP milestone.

We did have a functioning MVP by September 18, but as a team we did not think
that it was good enough.  We started reviewing all of the components of the
car, questioning aspects that we had previously constructed, and tried a
variety of new approaches to obtain better performance.  In the end this 
paid off and we were able to demonstrate better performance.  However, time
was running out and we had to prioritize what aspects we needed to work on
more than earlier in the project.

Our team predominately used Slack to facilitate communications, augmented
by e-mail correspondence and a few video conference calls used to meet each
other and check in on progress during the integration phase.  We opted to 
use a single GitHub repo with privileges granted all team members.  That
said, what was most important was being able to communicate asynchronously 
and be able to adapt to the requirements of team members when they had 
something beyond the project that needed their attention.  



### Lessons learned and areas for improvement

There are a number of lessons that were learned, both related to the 
technical aspects of the project as well as the way that we worked 
together to accomplish the goal.  The presentation here is chronological
spanning both topics.

It turned out to be very useful to be involved in a team early.  This 
allowed for accelerated milestones and helped keep the project on
track.  Early on we divided the work between the deep learning aspect
and the motion control aspect, with the expectation that as these were
completed we would all work across the entire project.  As we began,
those working on deep learning got familiar with the `ROSBAG` ideas, 
what the right model would be to obtain good results, how we would
generate this data.  At the same time, those working on the motion
control were exposed to other `ROS` features and learning about
controlling the project from a separate console. 

There was a lot of discussion early on about the model and how to 
generate the training data.  As well, there was a lot of discussion
about how to use the various motion control aspects that were 
provided (e.g. the yaw controller, PID controller, etc.), particularly
compared with reworking or replacing some of these.  Since we were one
of the first groups to have solid progress, at least that we could 
tell, we had to make a number of decisions by simply trying many 
potential solutions.  As it turned out, the two most troubling issues 
ended up being how to control the steering and how to generate and 
control the signal for braking and accelerating the car.  

Two ideas were put forward regarding the steering.  One was using the
stock yaw controller, which works very well except in the case 
of slowing down quickly, as the difference between the desired and 
current velocity affects the steering.  This solution is simple, 
essentially provided for us, but not ideal in this one case.  The 
other solution was to rework the yaw controller so that it was more
compatible with a PID controller, namely by having two yaw controllers,
on for the desired yaw and one for the current yaw, and then applying
a PID controller to this error term.  This was more complex, deviated
from the supplied code, and required using low pass filters for 
smoothing the resulting steering.  This also had negative properties,
as the low pass filter made performance in the tighter turns not as 
ideal.  We put forth models having both of these so that as a team
we could look at the relative performance and consider what would
be the best solution for the specific problem we are working on.

Regarding the control of the motion from the traffic light 
detection, there were three main ideas.  One was creating a minimum
jerk velocity profile for future waypoints and letting the PID
controller attempt to track this.  A second was to simply apply 
constant braking with magnitude determined by the stopping distance
and the characteristics of the car (mass, velocity, wheel radius),
and brake at the appropriate time before the light.  A third mechanism
was presented where the PID controller was still in the loop, and
utilized a soft-step mapping with asymptotic behavior so that effectively
it would brake hard when stopping and have a deterministic effect 
based on desired stopping distance, but for low speed behave 
gently.  Again we produced various branches employing the different
solutions so that we could choose a best solution. 

Much of this competitive solution optimization was performed after
we had an MVP on the date that we expected to have the MVP.  As a 
team we were simply not satisfied with the results and opted to 
work on opportunities for improvement that were consistent with the
vision we had for making the improvements.  Since everyone had their 
own ideas on this, we developed the ideas we liked best and then
started evaluating what worked best.

As you can see, the team dynamic and ability to work on different
parts of the problem, but come together at the end to make final
optimizations, was well played.  We did have issues with team
members having other engagements, with loosing sight of the larger
goal to focus on small details and revisiting solutions that we thought
we already solved, but the end result of coming up with the best
solution given all of the factors that contributed to the project
worked out successfully.

Of course there are areas for improvement, and lots of them.  Below are
listed just a few:

  * More training data
  * More types of augmentation
  * Interpolation of waypoints
  * Better tuning of PID controllers
  * Better coefficients for low pass filters
  * Evaluation of other methods for steering control
  * Evaluation of other methods of motion planning
  * Better estimation of traffic light for images
  * Wider range of speed with acceptable performance

While there are a lot of opportunities for improvement, we are also
happy about all of the parts of the project that we solved within
the time allowed for completion and within the scope of the objectives
that needed to be met for the solution.  While some of the aspects of 
the project were actually simpler than prior assignments, the larger,
integrated, scope of the project, as well as working in a team, 
gave us a better sense of the effort required to solve real world
problems in the autonomous vehicle space.  

Thank you so much, Udacity, for presenting us with the various 
problems over the last year, and for helping us become much more 
familiar with the practical challenges that go into the development
of self-driving vehicles.  It has been awesome.


