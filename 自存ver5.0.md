```c++
Angle calcAngleToGoal() const
  {
    struct ObstacleModel opponent;
    Vector2f opponent_pos (opponent.center.x(),opponent.center.y());
    const Angle opponent_to_robot = (theRobotPose.inversePose * opponent_pos).angle();
    if(abs(opponent_to_robot) <=  2_deg)  return 30_deg;

    else return 0_deg;
  }
```

