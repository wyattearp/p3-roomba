# Visualization code for simulated roomba robots.

import math
import time

try:    # Why do they have to change names?  Is there a better way to do this?
  from tkinter import *
except ImportError:
  from Tkinter import *


class RobotVisualization:
    def __init__(self, num_robots, room, delay = 0.2, goal = 1.0):
        "Initializes a visualization with the specified parameters."
        width = room.getWidth()
        height = room.getHeight()
        # Number of seconds to pause after each frame
        self.delay = delay

        self.max_dim = max(width, height)
        self.width = width
        self.height = height
        self.num_robots = num_robots
        self.paused = False
        self.quit = False
        self.goal = goal

        # Initialize a drawing surface
        self.master = Tk()
        self.w = Canvas(self.master, width=500, height=500)
        self.master.bind("<Key>", self.key)
        self.w.pack()
        self.master.update()

        # Draw a backing and lines
        x1, y1 = self._map_coords(0, 0)
        x2, y2 = self._map_coords(width, height)
        self.w.create_rectangle(x1, y1, x2, y2, fill = "white")

        # Draw gray squares for dirty tiles
        self.tiles = {}
        for i in range(width):
            for j in range(height):
                x1, y1 = self._map_coords(i, j)
                x2, y2 = self._map_coords(i + 1, j + 1)
                # Occupied
                if(room.isTileOccupied( (i,j) )):
                  self.tiles[(i, j)] = self.w.create_rectangle(x1, y1, x2, y2, 
                                                            fill = "black")
                # Dirty
                elif(room.isTileDirty( (i,j) )):
                  self.tiles[(i, j)] = self.w.create_rectangle(x1, y1, x2, y2,
                                                             fill = "gray")
                # Clean and drivable 
           #     else:
           #       self.tiles[(i, j)] = self.w.create_rectangle(x1, y1, x2, y2,
           #                                                  fill = "white")                                            
                                                             
                
        # Draw gridlines
        for i in range(width + 1):
            x1, y1 = self._map_coords(i, 0)
            x2, y2 = self._map_coords(i, height)
            self.w.create_line(x1, y1, x2, y2)
        for i in range(height + 1):
            x1, y1 = self._map_coords(0, i)
            x2, y2 = self._map_coords(width, i)
            self.w.create_line(x1, y1, x2, y2)

        # Draw some status text
        self.robots = None
        self.text = self.w.create_text(25, 0, anchor=NW,
                                       text=self._status_string(0, room))
        self.time = 0
        self.master.update()

    def _status_string(self, time, room):
        "Returns an appropriate status string to print."
        percent_clean = 100 *  float(room.getNumCleanTiles()) / room.getNumTiles()
        if self.paused:
          pause = 'PAUSED'
        else:
          pause = "'p' to pause"
        return "Time: %04d; %d tiles (%d%% goal %d%%) cleaned  %15s    'q' to quit" % \
            (time, room.getNumCleanTiles(), percent_clean, self.goal * 100, pause)

    def _map_coords(self, x, y):
        "Maps grid positions to window positions (in pixels)."
        return (250 + 450 * ((x - self.width / 2.0) / self.max_dim),
                250 + 450 * ((self.height / 2.0 - y) / self.max_dim))

    def _draw_robot(self, position, direction):
        "Returns a polygon representing a robot with the specified parameters."
        x, y = position
        d1 = direction + 165
        d2 = direction - 165
        x1, y1 = self._map_coords(x, y)
        x2, y2 = self._map_coords(x + 0.6 * math.sin(math.radians(d1)),
                                  y + 0.6 * math.cos(math.radians(d1)))
        x3, y3 = self._map_coords(x + 0.6 * math.sin(math.radians(d2)),
                                  y + 0.6 * math.cos(math.radians(d2)))
        return self.w.create_polygon([x1, y1, x2, y2, x3, y3], fill="red")
        
    def key(self,event):
      if event.char == 'p':
        self.paused = not self.paused
      if event.char == 'q':
        self.quit = True
        sys.exit(0)

    def update(self, room, robots):
        "Redraws the visualization with the specified room and robot state."
        # Removes a gray square for any tiles have been cleaned.
        for i in range(self.width):
            for j in range(self.height):
                if ((not room.isTileDirty( (i, j))) 
                  and not room.isTileOccupied( (i, j))
                  and (i,j) in self.tiles):
                    self.w.delete(self.tiles[(i, j)])
        # Delete all existing robots.
        if self.robots:
            for robot in self.robots:
                self.w.delete(robot)
                self.master.update_idletasks()
        # Draw new robots - This creates flicker if lots of robots
        self.robots = []
        for robot in robots:
            x,y = robot.robot.getRobotPosition()
            x1, y1 = self._map_coords(x - 0.2, y - 0.2)
            x2, y2 = self._map_coords(x + 0.2, y + 0.2)
            self.robots.append(self.w.create_oval(x1, y1, x2, y2,
                                                  fill = "black"))
            self.robots.append(
                self._draw_robot(robot.robot.getRobotPosition(), robot.robot.getRobotDirection()))
        # Update text
        self.w.delete(self.text)
        self.time += 1
        self.text = self.w.create_text(
            25, 0, anchor=NW,
            text=self._status_string(self.time, room))
        self.master.update()
        time.sleep(self.delay)
        # Loop if pause is enabled
        if self.paused:
          self.w.delete(self.text)
          self.text = self.w.create_text(
            25, 0, anchor=NW,
            text=self._status_string(self.time, room))
          while self.paused:
            time.sleep(0.1)
            self.master.update()

    def done(self):
        "Indicate that the animation is done so that we allow the user to close the window."
        mainloop()

