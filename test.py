import simulation as sim
import render as ren
import numpy as np

sim.loadEnvironment("data\plane.obj", thickness = 0.001, penetrationCoef = 1., elasticityCoef = 0., frictionResistanceCoef = 100., frictionCoef = 100, scale = 100., basePosition = np.array([0., 0., 0.]), baseOrientation = np.array([0., 0., 0.]))
sim.loadSnake("data\snake.veg", timestep = 0.01, dampingMassCoef = 0., dampingStiffnessCoef = 0.01, gravity = 10., muscleStiffnessCoef = 100000., basePosition = np.array([0., 0.1, 0.]), baseOrientation = np.array([0., 0., 0.]))

ren.createWindow(1080, 1080, "[Test] Snake Locomotion")

while ren.isOpenWindow():
    action = np.random.rand(sim.getNumMuscle())
    action = np.clip(action, 0.5, 1).astype(np.float64)

    for i in range(25):
        sim.contractMuscle(action)
        sim.simulate()
        ren.render()
