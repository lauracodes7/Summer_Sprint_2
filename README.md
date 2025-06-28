# Description:

In this sprint, your task is to create a new derived class that inherits from BaseMissionPlanner that implements the GetNextWaypoint() function.

The vehicle starts at a given waypoint with limited power and must reach a goal waypoint. Each move between waypoints consumes power (proportional to distance), and some waypoints can provide additional power when visited.

You must decide the logic for how the vehicle selects its next waypoint at each step. Your planner should:

- Use the provided Navigator2D to access waypoint position and power

- Avoid moving to any waypoint if the vehicle doesn't have enough power

- Try to reach the goal successfully before running out of power

You are free to choose any planning strategy, as long as it follows the interface and uses the available tools. The environment and mission trace will be visualized to help verify your solution.

