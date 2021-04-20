# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.



## Assignment result 

If you want to run this assignment, type in as follow:

```
python3 main.py
```

I make this assignment work, filling in the TODO in the file, markov_localizer.py. 

- def motion_model(position, mov, priors, map_size, stdev): This function is Motion model following 1-D Gaussian distribution

- def observation_model(landmarks, observations, pseudo_ranges, stdev): This funciton Observation model following indepedent Gaussian

The below is the resulting image to be captured when this assignmet is implemented.

![](https://github.com/hyunyoung2/vehicle-intelligence-2021/blob/master/code/week-2/markov_localization.png?raw=true)
