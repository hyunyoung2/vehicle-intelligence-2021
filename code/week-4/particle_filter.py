import numpy as np
from scipy.stats import multivariate_normal
import math
from helpers import distance

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations =  []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use 1self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        # 5. Update the particle's weight by the calculated probability.

        """
        this code refenced by https://github.com/vatsl/ParticleFilter/blob/master/src/particle_filter.cpp
        """       
 
        weight_sum = 0.0

        for p in self.particles:
            wt = 1.0
            
            landmarks = []
            for idx, val in map_landmarks.items():
                dist = distance(p, val)
                if dist < sensor_range:
                    landmarks.append({'id': idx, 'x': val['x'], 'y': val['y']})

            ##print("\nmap_lanmark: {}".format(len(map_landmarks)), map_landmarks)
            ##print("\nlandmarks: {}".format(len(landmarks)), landmarks) 

            if landmarks == []:
                #weight_sum += p['w']
                continue
            
            ## convert observation from vehicle's to map's coordinate system  
            linear_trans_x = [np.cos(p['t']), -np.sin(p['t']), p['x']]
            linear_trans_y = [np.sin(p['t']), np.cos(p['t']), p['y']]
            
            transformed_observations = []
            for idx, val in enumerate(observations):
                cur_coor = [val['x'], val['y'], 1]

                transformed_x = np.dot(cur_coor, linear_trans_x)
                transformed_y = np.dot(cur_coor, linear_trans_y)
                transformed_observations.append({'x': transformed_x, 'y': transformed_y})

            ##print("transformed_obs: {}".format(len(transformed_observations)), transformed_observations)

            ass = self.associate(landmarks, transformed_observations)
 
            ##print("\nass: {}".format(len(ass)), ass)
            
            ## for black point
            temp_id = []
            for i in range(len(ass)):
                temp_id.append(ass[i]['id'])

            p['assoc'] = temp_id
                         
         
            for idx, val in enumerate(transformed_observations):
              
                transformed_obs = {'x': transformed_x, 'y': transformed_y}
                
                single_landmark_k = []
                single_landmark = {}
 
                for idx, val in enumerate(landmarks):
                    dist = distance(transformed_obs, val)
                    single_landmark_k.append({'dist': dist, 'x': val['x'], 'y': val['y']})
                    

                distance_min = single_landmark_k[0]['dist']
                single_landmark = single_landmark_k[0]

                for i in range(1, len(single_landmark_k)):
                    if distance_min >= single_landmark_k[i]['dist']:
                        single_landmark = single_landmark_k[i]

                ##print("Single_landmark: ", single_landmark)
                ##// update weights using Multivariate Gaussian Distribution
                ##// equation given in Transformations and Associations Quiz


                normalizer = 1./ 2. * np.pi * std_landmark_x * std_landmark_y

                exponent = pow((transformed_obs['x'] - single_landmark['x']), 2) / pow(std_landmark_x, 2) + pow((transformed_obs['y'] - single_landmark['y']), 2) /pow(std_landmark_y, 2)

                obs_w = normalizer * math.exp((-0.5 * exponent))  
                obs_w +=  1e-25 # avoid round-off to zero

                wt *= obs_w
                
                ##print("Wt: ", wt)

            #weight_sum += wt
            p['w'] = wt


        ## normalize weight to bring them in (0, 1]
        ##test1 = []
        ##test = []
        ##for _p in self.particles:
        ##    test1.append(_p['w'])
        ##    _p['w'] /= weight_sum
        ##    test.append(_p['w'])

        ##print("\ntest1: ", test1)
        ##print(weight_sum)
        ##print("\ntest: ", test)
        ##input() 
         
            
    # Resample particles with replacement with probability proportional to
    #   their weights.
    def resample(self):
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.

        """
         refereced by https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.rv_discrete.html
        """
        import copy 
        from scipy import stats

        weights = [p['w'] for p in self.particles]

        ## normalize weight to bring them in (0, 1]
        pk = weights/np.sum(weights)

        ##print("\nweight: {}".format(len(weights)), weights) 
        ##print("\nweight_sum: {}".format(np.sum(weights)))
        ##print("\npk: {}".format(pk))

        xk = np.arange(len(self.particles))

        ##print("\nParticles: {}".format(len(self.particles)), self.particles)

        custm = stats.rv_discrete(name="paticles", values=(xk, pk))

        resampled_idx = custm.rvs(size=self.num_particles)

        ##print("\nresample_idx: {}".format(len(resampled_idx)), resampled_idx)
        resampled_particles = []

        for idx in resampled_idx:
             resampled_particles.append(copy.deepcopy(self.particles[idx]))

        self.particles = resampled_particles
          
        ##input()       
        

 
    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle



