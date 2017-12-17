# Model Predictive Control

## The Model

Our model is a simple kinematic bicycle model.

### State
The state consists of 6 values: x and y position, heading angle (psi), velocity, and two error terms, cross-track-error (cte) and heading-angle-error, (epsi).

Value x, y, psi, and v are obtained from measurements (here, the simulator), and we calculate cte and epsi from these values using waypoints provided by the simulator.

### Actuators
We only have two actuators, acceleration (a) and steering (delta). Acceleration covers braking in addition to speeding up, but for my model I set a lower limit of 0 to prevent active deceleration. The car can still slow down by simply letting off the gas, which is sufficient. Steering angles are limited in the simulator to 25 degrees, but I capped my steering values at 19 degrees because the 25 was unnecessary.

### Update Equations
The update equations are standard for a simple kinematic model. This includes a preset Lf scaling value for velocity and turning to accurately calculate epsi.

## Timestep Length and Elapsed Duration (N & dt)
Two key values to tune in MPC are the timestep length (dt) and the iteration count (N). These determine how far into the future the MPC looks and how fine-grained the predictions will be (every 1 meter, or every 10 meters?).

At first I chose value too large for N, around 30. This cause performance issues and was causing the MPC to predict unnecessarily far into the future. I ended up settling on an N value of 10, which works well.

I used many different dt values over the course of my tweaking. I found that this value needed to be decreased as the reference velocity increased so that an appropriate prediction distance was maintained. At a low speed of 10mph, 0.5 was sufficient. I settled on a reference velocity of 30mph, so dt of 0.1 is what I ended up with.


## Polynomial Fitting and MPC Preprocessing
To run MPC, we need a polynomial fitted to waypoints as a reference trajectory. I chose to fit a 3rd degree polynomial.

Before fitting the polynomial and passing state values to the MPC solver, I converted all values into the car's frame of reference. I derived formulas for converting cartesian coordinates in one frame of reference to another and then passed the waypoints from the simulator through this function. I also had to tweak the state values for the car that came back from the simulator so they would also be in the car's frame of reference. Thankfully this part was easy, as most values just became 0.

I believe converting to the car's frame of reference made calculating the error values easier and probably more accurate. It was also handy because the reference lines the simulator draws are all points from the car's frame of reference.

## Latency
The latency was a little tricky to deal with at first, so I turned it off. Once I got my MPC working without the latency, I turned it back on and found that I needed to do some tweaking. I'm not really sure why, but the latency affected the lines the simulator drew, rotating them relative to the car so they were out of place. Perhaps this is because the lines are drawn relative to the car which had an additional 100ms of travel before the draw?

Tweaking cost scaling values took care of the latency problem for me. Specifically, increasing the cost of high steer angle values helped the most. Issues still show up at times causing the car to oscillate, but it slowly settles down and doesn't get out of control.
