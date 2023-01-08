import pybullet as p
import time

t = 1/60
physicsClient = p.connect(p.GUI)
for i in range(1000):
  p.stepSimulation()
  time.sleep(t)
  print(i)

p.disconnect()
