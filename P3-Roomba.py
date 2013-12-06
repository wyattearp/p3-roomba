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
    else:
      self.action = ('Forward',None)

class Chromosome():
  all_possibles = range(0,360)
  current_list_of_possibles = all_possibles

  def getNextChoices(self,previous_list):
    print("Pruning list:")
    print(previous_list)
    # this function will likely be wack
    # keep top 5%
    # drop bottom 20%
    # change out XX%
    # TODO: these statics will break if we make it all the way through the list
    keep_list = previous_list[0]
    bottom_list = previous_list[6:10]
    remainder_list = previous_list[1:6]

    # clear our previous list out of the list of possible to preven rework
    self.current_list_of_possibles = list(set(self.current_list_of_possibles) - set(previous_list))
    # for each of the remainders that didn't get dropped, mutate them closer
    mutants_list = []
    for i in remainder_list:
      # march each item closer
      value = i
      if value > keep_list:
        value -= 1
      elif value < keep_list:
        value += 1
      elif value == keep_list:
        value = None
      # only add the item if its a new thing
      if value:
        mutants_list.append(value)

    # clear the mutants_list out of the list
    self.current_list_of_possibles = list(set(self.current_list_of_possibles) - set(mutants_list))

    # we're keeping the top guy, so at most we can replace would be 9
    need_to_replace = 9 - len(mutants_list)
    new_breed_list = []
    if (need_to_replace != 0):
        # we need to add this many items to ones we've lost
        print("Breeding %d new items") % (need_to_replace)
        new_breed_list = self.getNewChoices(need_to_replace,self.current_list_of_possibles)

    # remove our new breed from the current_list_of_possibles
    self.current_list_of_possibles = list(set(self.current_list_of_possibles) - set(new_breed_list))

    # return the contenders
    new_list = [keep_list] + mutants_list + new_breed_list
    print("New list of contenders:")
    print(new_list)
    return new_list


  def getNewChoices(self,numChoices,list_of_choices):
    choices = []
    if(len(list_of_choices) >= numChoices):
      for b in range(0,numChoices):
        position = random.randint(0,len(list_of_choices)-1)
        choices.append(list_of_choices[position])
    else:
      choices = list_of_choices

    print("Returning %d new choices") % numChoices
    print(choices)

    return choices

  def getChromosome(self,rooms, start_location, min_clean):
      numRooms = len(rooms)
      unsorted_list = []

      with Timer() as cd:
        initTime = cd.start

        possibles = self.getNewChoices(10,self.all_possibles)
        print("Starting with:")
        print(possibles)

        while ((time.clock() - initTime) <= 50.0 and len(possibles) > 1):
          print("%f seconds left...") % (50.0 - (time.clock() - initTime))
          result_list = []
          run_result_list = []
          for p in possibles:
            print("P: %d") % p
            for r in rooms:
              result = runSimulation(num_robots = 1,
                          min_clean = min_clean,
                          start_location = start_location,
                          num_trials = 1,
                          room = r,
                          robot_type = TunedRobot,
                          #ui_enable = True,
                          ui_delay = 0.1,
                          chromosome = p)
              
              run_result_list.append(result[0])
            # generate the average run for this item
            average = sum(run_result_list) / len(run_result_list)
            # store the average for later comparison
            result_list.append((average,p))
          # now sort the averages
          sorted_list = sorted(result_list, key=lambda tup: tup[0])
          # take the list of results and get the next ones
          last_run_list = zip(*sorted_list)[1]
          print("Results of the last run:")
          print(sorted_list)

          possibles = self.getNextChoices(last_run_list)
          print("Retrying with the next list")
          print(possibles)
        # looks like we're out of time, return the best of the best
        return possibles[0]

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
    c = Chromosome()
    chromosome = c.getChromosome(rooms, startLoc, minClean)
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





