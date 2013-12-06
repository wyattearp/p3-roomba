# H1.py
# Author: Paul Talaga
#
# This file demonstrates how to implement various kinds of Roomba robot agents
# and run them in GUI or non-gui mode using the roomba_sim library.
#

from roomba_sim import *
from roomba_concurrent import *

# Each robot below should be a subclass of ContinuousRobot, RealisticRobot, or DiscreteRobot.
# All robots need to implement the runRobot(self) member function, as this is where
# you will define that specific robot's characteristics.

# All robots perceive their environment through self.percepts (a class variable) and
# act on it through self.action.  The specific percepts received and actions allowed
# are specific to the superclass.  See roomba_sim for details.

# TunedRobot - Robot that acceps a chromosome parameter and can store limited state
#               (a single number).  Continuous and dynamic environment.

import time
import random

class Timer:
  def __enter__(self):
    self.start = time.clock()
    return self

  def __exit__(self, *args):
    self.end = time.clock()
    self.interval = self.end - self.start

class TunedRobotDefault(RealisticRobot):
  """ The ReflexRobotState robot is similar to the ReflexRobot, but some
    state is allowed in the form of a number.
  """
  def __init__(self,room,speed, start_location = -1, chromosome = None):
    super(TunedRobotDefault, self).__init__(room,speed, start_location)
    # Set initial state here you may only store a single number.
    self.state = 0
    # Save chromosome value
    self.degrees = chromosome


  def runRobot(self):
    (bstate, dirt) = self.percepts
    if(bstate == 'Bump'):
      self.action = ('TurnRight',135 + self.degrees)
    elif(dirt == 'Dirty'):
      self.action = ('Suck',None)
    else:
      self.action = ('Forward',None)

class TunedRobot(RealisticRobot):
  """ The ReflexRobotState robot is similar to the ReflexRobot, but some
    state is allowed in the form of a number.
  """
  def __init__(self,room,speed, start_location = -1, chromosome = None):
    super(TunedRobot, self).__init__(room,speed, start_location)
    # Set initial state here you may only store a single number.
    self.state = 0
    # Save chromosome value
    self.degrees = chromosome


  def runRobot(self):
    (bstate, dirt) = self.percepts
    if(bstate == 'Bump'):
      self.action = ('TurnRight',135 + self.degrees)
    elif(dirt == 'Dirty'):
      self.action = ('Suck',None)
    elif(self.state > 4):
      self.action = ('TurnRight',135 + self.degrees)
      self.state = 0
    else:
      self.action = ('Forward',None)
      self.state += 1

all_possibles = range(0,360)
current_list_of_possibles = all_possibles
max_return = 10

def getNextChoices(previous_list):
  # this function will likely be wack
  # keep top 5%
  # drop bottom 20%
  # change out X%
  keep_list = previous_list[0]
  bottom_list = previous_list[6:10]
  remainder_list = previous_list[1:6]
  current_list_of_possibles = set(current_list_of_possibles) - set(keep_list)
  current_list_of_possibles = set(current_list_of_possibles) - set(bottom_list)
  # so there are 4 items left in the original
  # randomly pick one of them and put it back in the list of possibles
  # randomly pick from the list of possibles and put it in place of the previous item




def getChromosome(rooms, start_location, min_clean):
    numRooms = len(rooms)
    unsorted_list = []

    with Timer() as cd:
      initTime = cd.start
      c = 0;

      while ((time.clock() - initTime) <= 50.0 and c < 361):
        print("%f seconds left...") % (50.0 - (time.clock() - initTime))
        # pick a number
        # TODO: be smarter
        c = c+1
        average_time = 0.0
        idx = 0
        timerArray = []

        for r in rooms:
          with Timer() as t:
            result = runSimulation(num_robots = 1,
                        min_clean = min_clean,
                        start_location = start_location,
                        num_trials = 1,
                        room = r,
                        robot_type = TunedRobot,
                        #ui_enable = True,
                        ui_delay = 0.1,
                        chromosome = c)
          timerArray.append(result[0])
          idx += 1
        # get the score to solve a room
        # TODO: replace with std dev?
        average_time = sum(timerArray) / long(len(timerArray))
        unsorted_list.append((average_time,c))
        #print("%d: provied average of %0.03f sec.") % (c,average_time)
    # now sort the averages
    sorted_list = sorted(unsorted_list, key=lambda tup: tup[0])
    return sorted_list[0][1]

############################################
## A few room configurations

allRooms = []

smallEmptyRoom = RectangularRoom(10,10)
allRooms.append(smallEmptyRoom)  # [0]

largeEmptyRoom = RectangularRoom(10,10)
allRooms.append(largeEmptyRoom) # [1]

mediumWalls1Room = RectangularRoom(30,30)
mediumWalls1Room.setWall((5,5), (25,25))
allRooms.append(mediumWalls1Room) # [2]

mediumWalls2Room = RectangularRoom(30,30)
mediumWalls2Room.setWall((5,25), (25,25))
mediumWalls2Room.setWall((5,5), (25,5))
allRooms.append(mediumWalls2Room) # [3]

mediumWalls3Room = RectangularRoom(30,30)
mediumWalls3Room.setWall((5,5), (25,25))
mediumWalls3Room.setWall((5,15), (15,25))
mediumWalls3Room.setWall((15,5), (25,15))
allRooms.append(mediumWalls3Room) # [4]

mediumWalls4Room = RectangularRoom(30,30)
mediumWalls4Room.setWall((7,5), (26,5))
mediumWalls4Room.setWall((26,5), (26,25))
mediumWalls4Room.setWall((26,25), (7,25))
allRooms.append(mediumWalls4Room) # [5]

mediumWalls5Room = RectangularRoom(30,30)
mediumWalls5Room.setWall((7,5), (26,5))
mediumWalls5Room.setWall((26,5), (26,25))
mediumWalls5Room.setWall((26,25), (7,25))
mediumWalls5Room.setWall((7,5), (7,22))
allRooms.append(mediumWalls5Room) # [6]

#############################################
def TunedTest1():
  print(runSimulation(num_robots = 1,
                    min_clean = 0.95,
                    num_trials = 1,
                    room = allRooms[6],
                    robot_type = TunedRobot,
                    #ui_enable = True,
                    ui_delay = 0.1,
                    chromosome = 0))

def TunedTest2():
  print(runSimulation(num_robots = 1,
                    min_clean = 0.95,
                    num_trials = 1,
                    room = allRooms[6],
                    robot_type = TunedRobot,
                    #ui_enable = True,
                    ui_delay = 0.1,
                    chromosome = 2))

if __name__ == "__main__":
  # This code will be run if this file is called on its own
  #TunedTest1()
  #TunedTest2()

  # This is an example of how we will test your program.  Our rooms will not be those listed above, but similar.
  rooms = [allRooms[1], allRooms[5]]
  startLoc = (5,5)
  minClean = 0.2
  random.seed()
  with Timer() as gcTime:
    chromosome = getChromosome(rooms, startLoc, minClean)
  print("getChromosome took %00.03f sec") % gcTime.interval


  # Concurrent test execution.
  myTunedRobotResult = concurrent_test(TunedRobot, rooms, num_trials = 20, min_clean = minClean, chromosome = chromosome)
  reactiveAgentResult = concurrent_test(TunedRobotDefault, rooms, num_trials = 20, min_clean = minClean, chromosome = 0)

  if (myTunedRobotResult < reactiveAgentResult):
    print("Chromosome %d did better (by %0.0f%%) than a simple reactive agent, %f < %f") % (chromosome, (100 - (myTunedRobotResult / reactiveAgentResult)*100), myTunedRobotResult,reactiveAgentResult)
  elif (myTunedRobotResult > reactiveAgentResult):
    print("Chromosome %d sucked (but in a bad way) more than a simple reactive agent, %f > %f") % (chromosome,myTunedRobotResult,reactiveAgentResult)
  elif (myTunedRobotResult == reactiveAgentResult):
    # this probably shouldn't be possible
    print("Weirdness Alert! Chromosome %d tied simple reactive agent, %f == %f") % (chromosome,myTunedRobotResult,reactiveAgentResult)





