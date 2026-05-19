import os
import gym
from gym import spaces, logger
from gym.utils import seeding
import gym.envs.custom.simulation as sim
import gym.envs.custom.render as ren
import numpy as np

# Load
dataPath = os.path.dirname(__file__) + "\\data\\"
sim.loadEnvironment(dataPath + "plane.obj", thickness = 0.001, penetrationCoef = 1., elasticityCoef = 0., frictionResistanceCoef = 100., frictionCoef = 100, scale = 100., basePosition = np.array([0., 0., 0.]), baseOrientation = np.array([0., 0., 0.]))
sim.loadSnake(dataPath + "snake.veg", timestep = 0.01, dampingMassCoef = 0., dampingStiffnessCoef = 0.01, gravity = 10., muscleStiffnessCoef = 100000., basePosition = np.array([0., 0.1, 0.]), baseOrientation = np.array([0., 0., 0.]))

# Snake Locomotion
class SnakeLocomotion(gym.Env):
    metadata = { 'render.modes': ['human'] }

    def __init__(self):
        # Map
        self.minMapBoundary = np.array([-100, -100, -100], dtype='float64')
        self.maxMapBoundary = np.array([100, 100, 100], dtype='float64')
        
        # Goal
        self.goalPosition = np.array([0, 0, 0], dtype='float64')
        self.goalDistance = 0.25
        
        self.isRandomGoal = True
        self.minGoalBoundary = np.array([-10, 0, -10], dtype='float64')
        self.maxGoalBoundary = np.array([10, 0, 10], dtype='float64')
        
        # Muscle
        self.minMuscleContractRate = 0.5
        self.maxMuscleContractRate = 1.0

        # Simulation
        self.numStep = 25

        # Render
        self.viewer = True
        
        # Seed
        self.seed()



        # State
        self.numCube = int(sim.getNumVertex() / 10)

        self.direction = None
        self.position = None
        self.velocity = None
        self.state = None

        # Reward
        self.distance = None
        
        # Action Space
        self.action_space = spaces.Box(self.minMuscleContractRate, self.maxMuscleContractRate, shape=(sim.getNumMuscle(), ), dtype='float64')
        
        # Observation Space
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(3 + 2 * 3 * 4 * (self.numCube + 1), ), dtype='float64')

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # Action
        action = np.clip(action, self.minMuscleContractRate, self.maxMuscleContractRate).astype(np.float64)

        for i in range(self.numStep):
            sim.contractMuscle(action)
            sim.simulate()
            
            if self.viewer:
                self.render()

        # State
        pos = sim.getVertexPosition()
        vel = sim.getVertexVelocity()
        ori = np.ones(3 * (self.numCube + 1)).reshape(self.numCube + 1, 3) * pos[7]
        mat = sim.getInverseLocalFrame()

        self.direction = normal(mat @ (self.goalPosition - pos[7]))
        pos0 = ((mat @ ((pos[0::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        pos1 = ((mat @ ((pos[1::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        pos3 = ((mat @ ((pos[3::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        pos4 = ((mat @ ((pos[4::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        self.position = np.concatenate([pos0, pos1, pos3, pos4])
        vel0 = ((mat @ ((vel[0::10]).T)).T).reshape(3 * (self.numCube + 1))
        vel1 = ((mat @ ((vel[1::10]).T)).T).reshape(3 * (self.numCube + 1))
        vel3 = ((mat @ ((vel[3::10]).T)).T).reshape(3 * (self.numCube + 1))
        vel4 = ((mat @ ((vel[4::10]).T)).T).reshape(3 * (self.numCube + 1))
        self.velocity = np.concatenate([vel0, vel1, vel3, vel4])
        self.state = np.concatenate([self.direction, self.position, self.velocity])

        # Reward
        reward = 0

        goalTheta = np.arccos(self.direction[0])
        upVector = sim.getLocalFrame()[1]
        upTheta = np.arccos(upVector[1])
        currentDistance = length(self.goalPosition - pos[7])
        differentDistance = self.distance - currentDistance

        if goalTheta < (np.pi / 4) and upTheta < (np.pi / 4) and differentDistance > 0:
            reward = self.direction[0] * upVector[1] * np.exp(differentDistance * 10)
        else:
            if goalTheta > (np.pi / 4):
                reward = reward - (goalTheta / np.pi)
            if upTheta > (np.pi / 4):
                reward = reward - (upTheta / np.pi)
            if differentDistance < 0:
                reward = reward + (differentDistance / 10)
                
        self.distance = currentDistance
        reward = reward / 1000

        # Done
        done = False
        if pos[7][0] < self.minMapBoundary[0] or pos[7][1] < self.minMapBoundary[1] or pos[7][2] < self.minMapBoundary[2]:
            done = True
        if pos[7][0] > self.maxMapBoundary[0] or pos[7][1] > self.maxMapBoundary[1] or pos[7][2] > self.maxMapBoundary[2]:
            done = True
        if self.distance < self.goalDistance:
            done = True

        return np.array(self.state), reward, done, {}

    def reset(self):
        # Reset
        sim.resetSimulation()
        
        # Reset Goal
        self.goalPosition[0] = self.np_random.uniform(low=self.minGoalBoundary[0], high=self.maxGoalBoundary[0], size=(1,))
        self.goalPosition[1] = self.np_random.uniform(low=self.minGoalBoundary[1], high=self.maxGoalBoundary[1], size=(1,))
        self.goalPosition[2] = self.np_random.uniform(low=self.minGoalBoundary[2], high=self.maxGoalBoundary[2], size=(1,))
        sim.setGoal(self.goalPosition[0], self.goalPosition[1], self.goalPosition[2])

        # Reset State
        pos = sim.getVertexPosition()
        vel = sim.getVertexVelocity()
        ori = np.ones(3 * (self.numCube + 1)).reshape(self.numCube + 1, 3) * pos[7]
        mat = sim.getInverseLocalFrame()

        self.direction = normal(mat @ (self.goalPosition - pos[7]))
        pos0 = ((mat @ ((pos[0::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        pos1 = ((mat @ ((pos[1::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        pos3 = ((mat @ ((pos[3::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        pos4 = ((mat @ ((pos[4::10] - ori).T)).T).reshape(3 * (self.numCube + 1))
        self.position = np.concatenate([pos0, pos1, pos3, pos4])
        vel0 = ((mat @ ((vel[0::10]).T)).T).reshape(3 * (self.numCube + 1))
        vel1 = ((mat @ ((vel[1::10]).T)).T).reshape(3 * (self.numCube + 1))
        vel3 = ((mat @ ((vel[3::10]).T)).T).reshape(3 * (self.numCube + 1))
        vel4 = ((mat @ ((vel[4::10]).T)).T).reshape(3 * (self.numCube + 1))
        self.velocity = np.concatenate([vel0, vel1, vel3, vel4])
        self.state = np.concatenate([self.direction, self.position, self.velocity])

        # Reset Reward
        self.distance = length(self.goalPosition - pos[7])
        
        return np.array(self.state)

    def render(self, mode='human'):
        if not ren.isOpenWindow():
            ren.createWindow()
            
        ren.render()

    def close(self):
        ren.destroyWindow()

def length(vec):
        return np.sqrt(np.dot(vec, vec))
    
def normal(vec):
        return vec / length(vec)
