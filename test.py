import simulation as sim
import render as ren
import numpy as np

sim.loadEnvironment("data\plane.obj", thickness = 0.001, penetrationCoef = 1., elasticityCoef = 1., frictionResistanceCoef = 1., frictionCoef = 1., scale = 100., basePosition = np.array([0., 0., 0.]), baseOrientation = np.array([0., 0., 0.]))
sim.loadSnake("data\snake.veg", timestep = 0.001, dampingMassCoef = 1., dampingStiffnessCoef = 1., gravity = 10., muscleStiffnessCoef = 0., basePosition = np.array([0., 1., 0.]), baseOrientation = np.array([0., 0., 0.]))
ren.createWindow(1080, 1080, "[Test] Snake Locomotion")

while ren.isOpenWindow():
    action = np.random.rand(sim.getNumMuscle())
    action = np.clip(action, 0.5, 1).astype(np.float64)
    sim.contractMuscle(action)
    sim.simulate()
    ren.render()
