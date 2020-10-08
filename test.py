import simulation as sim
import render as ren
import numpy as np

sim.loadEnvironment("data\plane.obj", thickness = 0.001, penetrationCoef = 1, elasticityCoef = 0, frictionResistanceCoef = 0, frictionCoef = 0,
                    scale = 100., basePosition = np.array([0., 0., 0.]), baseOrientation = np.array([0., 0., 0.]))

sim.loadSnake("data\snake.veg", timestep = 0.01, dampingMassCoef = 0., dampingStiffnessCoef = 0.1, gravity = 10., muscleStiffnessCoef = 1000.,
              basePosition = np.array([0., 1., 0.]), baseOrientation = np.array([0., 0., 0.]))

ren.createWindow()

while True:
    sim.simulate()
    ren.render()
