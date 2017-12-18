# Model Predictive Control

## The Model

Our model is a simple kinematic bicycle model.

### State
The state consists of 6 values: x and y position, heading angle (psi), velocity, and two error terms, cross-track-error (cte) and heading-angle-error, (epsi).

Value x, y, psi, and v are obtained from measurements (here, the simulator), and we calculate cte and epsi from these values using waypoints provided by the simulator.

### Actuators
We only have two actuators, acceleration (a) and steering (delta). Acceleration covers braking in addition to speeding up, but for my model I set a lower limit of 0 to prevent active deceleration. The car can still slow down by simply letting off the gas, which is sufficient.

### Update Equations
The update equations are standard for a simple kinematic model. This includes a preset Lf scaling value for velocity and turning to accurately calculate epsi.

## Timestep Length and Elapsed Duration (N & dt)
Two key values to tune in MPC are the timestep length (dt) and the iteration count (N). These determine how far into the future the MPC looks and how fine-grained the predictions will be (every 1 meter, or every 10 meters?).

At first I chose value too large for N, around 30. This cause performance issues and was causing the MPC to predict unnecessarily far into the future. I ended up settling on an N value of 10, which works well.

I used many different dt values over the course of my tweaking. I found that this value needed to be decreased as the reference velocity increased so that an appropriate prediction distance was maintained. At a low speed of 10mph, 0.5 was sufficient. I was able to achieve a reference velocity of 80mph, so a dt of 0.15 is what I ended up with.


## Polynomial Fitting and MPC Preprocessing
To run MPC, we need a polynomial fitted to waypoints as a reference trajectory. I chose to fit a 3rd degree polynomial.

Before fitting the polynomial and passing state values to the MPC solver, I converted all values into the car's frame of reference. I derived formulas for converting cartesian coordinates in one frame of reference to another and then passed the waypoints from the simulator through this function. I also had to tweak the state values for the car that came back from the simulator so they would also be in the car's frame of reference. Thankfully this part was easy, as most values just became 0.

I believe converting to the car's frame of reference made calculating the error values easier and probably more accurate. It was also handy because the reference lines the simulator draws are all points from the car's frame of reference.

## Latency
The latency was a little tricky to deal with at first, so I turned it off. Once I got my MPC working without the latency, I turned it back on and found that I needed to do some tweaking.

While altering the cost factors helped with the latency issues, the biggest improvement came when I pre-forecasted where the car would be 100ms later before passing state values to the MPC solver. I used the same kinematic equations in the solver to change the initial state (provided by the simulator) to be where we expect the car to be 100ms in the future.
