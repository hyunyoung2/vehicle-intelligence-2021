# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.



## Assignment result 

If you want to run this assignment, type in as follow:

```
python3 run.py
```

I make this assignment work, filling in the TODO in the file, particle_filter.py . 

-  def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks): This function is to update the weights of each particle using a multi-variate Gaussian distribution.

-  def resample(self): This funciton is to resample particles with replacement with probability proportional to their weights.

The below is the resulting image to be captured when this assignmet is implemented.

![](https://github.com/hyunyoung2/vehicle-intelligence-2021/blob/master/code/week-4/particle_filter-1.png?raw=true)
