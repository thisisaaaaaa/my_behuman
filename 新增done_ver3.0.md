```c++
Angle calcAngleToGoal() const
{
  // 基础角度：指向球门中心
  const Vector2f goalCenter(theFieldDimensions.xPosOpponentGroundline, 0.f);
  const Angle baseAngle = (theRobotPose.inversePose * goalCenter).angle();

  // 寻找球门区域内的守门员
  bool hasGoalkeeper = false;
  float closestY = 0.f;  // 记录离球门中心最近的守门员横向位置

  for(const auto& obstacle : theObstacleModel.obstacles)
  {
    if(obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent)
    {
      // 转换到全局坐标系
      const Vector2f globalPos = theRobotPose * obstacle.center;
      
      // 检查是否在球门区域
      if(globalPos.x() > theFieldDimensions.xPosOpponentGroundline - 500.f && 
         std::abs(globalPos.y()) < goalDetectionWidth / 2)
      {
        // 取离球门中心最近的守门员
        if(!hasGoalkeeper || std::abs(globalPos.y()) < std::abs(closestY))
        {
          hasGoalkeeper = true;
          closestY = globalPos.y();
        }
      }
    }
  }

  // 没有守门员时返回原始角度
  if(!hasGoalkeeper)
    return baseAngle;

  // 简化的角度调整：根据守门员位置向左/右偏移固定角度
  const Angle offset = (closestY > 0) ? -maxAvoidanceAngle : maxAvoidanceAngle; // 守门员在右侧则向左偏
  return baseAngle + offset;
}
```

```c++
class CodeReleaseKickAtGoalCard : public CodeReleaseKickAtGoalCardBase
{
  // ... 保持 preconditions/postconditions 不变 ...

  option
  {
    theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);

    // 新增状态：完成踢球后保持不动
    state(done)
    {
      action
      {
        theStandSkill();
      }
    }

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
      }
      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToBall;
      }
      action { /* 原有动作不变 */ }
    }

    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }
      action { /* 原有动作不变 */ }
    }

    state(alignToGoal)
    {
      // 强化对准条件：更严格的角度阈值
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < 3_deg &&  // 原为 angleToGoalThreshold (10_deg)
           std::abs(theFieldBall.positionRelative.y()) < 50.f)  // 原为 ballYThreshold (100.f)
          goto alignBehindBall;
      }
      action { /* 原有动作不变 */ }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        // 强化最终对准条件
        if(std::abs(angleToGoal) < 1_deg &&  // 原为 angleToGoalThresholdPrecise (2_deg)
           ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) &&
           ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }
      action { /* 原有动作不变 */ }
    }

    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        // 踢球完成后进入 done 状态，不再循环
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto done;  // 原为 goto start
      }
      action { /* 原有动作不变 */ }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }
      action { /* 原有动作不变 */ }
    }
  }

  Angle calcAngleToGoal() const
  {
    // 保持原有避障逻辑不变
    // ...
  }
};
```

