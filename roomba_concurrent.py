# roomba_concurrent.py
# Author: Dan Grahn
#
# This file provides a method to run Roomba Simulation tests in a concurrent
# environment. It uses multiple processes instead of threads because of
# Python's global interpretter lock.

from roomba_sim import *
from multiprocessing import Process, Manager
import time

class SimulationProcess(Process):
    """
    A process which will run the simulation when started.
    """

    def __init__(self, num, dict):
        Process.__init__(self)
        self.num = num
        self.dict = dict
    #end __init__

    def run(self):
        result = runSimulation(num_robots = 1,
                               speed = 1,
                               min_clean = self.min_clean,
                               num_trials = self.num_trials,
                               room = self.room,
                               robot_type = self.robot,
                               start_location = self.start_location,
                               chromosome = self.chromosome,
                               ui_enable = False)
        self.dict[self.num] = result
    #end run
    
    def join(self, timeout = None):
        Process.join(self, timeout)
    #end join
#end SimulationProcess

def concurrent_test(robot, rooms, num_trials, start_location = -1, min_clean = 1.0, chromosome = None, timeout = 5*60):
    """
    Run the tests in multiple processes. Can be directly swapped out for testAllMaps.
    """
    # Setup variables
    num_rooms    = len(rooms)               # Total number of rooms
    total_trials = num_trials * num_rooms   # Total number of trials
    processes    = []                       # List for all processes
    manager      = Manager()                # Manager to handle result transfer
    dict         = manager.dict()           # Dict which will store results
    
    # Create a process for each room, storing parameters in instance variables
    for i, room in enumerate(rooms):
        process = SimulationProcess(i, dict)
        process.robot          = robot
        process.room           = room
        process.num_trials     = num_trials
        process.start_location = start_location
        process.min_clean = min_clean
        process.chromosome     = chromosome
        process.start()
        processes.append(process)
    #end for

    # Wait for timeout, or whenever everything is done
    starttime = time.time()
    alldone = True
    while time.time() - starttime < timeout:
      alldone = True
      for i, process in enumerate(processes):
        if process.is_alive():
          alldone = False
      if alldone:
        break
      time.sleep(1)
    
    # See if processes need killing
    if not alldone:
      print('Timeout of ' + str(timeout) + ' seconds occured, killing processes...')
      for i, process in enumerate(processes):
        if process.is_alive():
          print('  Killed ' + str(i))
          process.terminate()
          dict[i] = (99998,0)
        
    # Print the results
    total_score = 0
    for i, process in enumerate(processes):
        process.join()
        (score, std) = dict[i]
        print("Run %d of %d done (score: %d std: %d)" % (i + 1, num_rooms, score, std))
        total_score += score
    #end for
    
    print("Average score over %d trials: %d" % (total_trials, total_score / num_rooms))
    return total_score / num_rooms
#end concurrent_test
