```c++
/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Modeling/ObstacleModel.h"    // 新增障碍物模型

CARD(CodeReleaseKickAtGoalCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(ObstacleModel),//障碍物
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(500.f) ballNearThreshold,
    (Angle)(10_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(40.f) ballOffsetY,
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (float)(300.f) goalkeeperAvoidanceRadius,  // 守门员避障半径
    (Angle)(100_deg) maxAvoidanceAngle,         // 最大射门角度偏移量
    (float)(2200.f) goalDetectionWidth,        // 球门有效区域宽度（检测守门员是否在射门路径上）
  }),
});

class CodeReleaseKickAtGoalCard : public CodeReleaseKickAtGoalCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);

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

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
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

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < 3_deg && std::abs(theFieldBall.positionRelative.y()) < 50.f)
          goto alignBehindBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < 1_deg && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto done;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }

  
  Angle calcAngleToGoal() const
  {
    // 基础角度：指向球门中心
    const Vector2f goalCenter(theFieldDimensions.xPosOpponentGroundline, 0.f);
    const Angle baseAngle = (theRobotPose.inversePose * goalCenter).angle();
  
    // 获取球门左右边界角度
    const Vector2f leftPost(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal);
    const Vector2f rightPost(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal);
    const Angle leftAngle = (theRobotPose.inversePose * leftPost).angle();
    const Angle rightAngle = (theRobotPose.inversePose * rightPost).angle();
  
    // 寻找最具威胁的守门员（离球门中心最近）
    float maxLateralDistance = 0.f;
    bool hasGoalkeeper = false;
    
    for(const auto& obstacle : theObstacleModel.obstacles)
    {
      if(obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent)
      {
        const Vector2f globalPos = theRobotPose * obstacle.center;
        
        // 扩展检测区域到整个球门范围
        if(globalPos.x() > theFieldDimensions.xPosOpponentGroundline - 1000.f)
        {
          const float lateralDist = std::abs(globalPos.y());
          if(lateralDist > maxLateralDistance)
          {
            maxLateralDistance = lateralDist;
            hasGoalkeeper = true;
          }
        }
      }
    }
  
    if(!hasGoalkeeper)
      return baseAngle;
  
    // 动态计算偏移比例（守门员越靠边，偏移越大）
    const float goalHalfWidth = theFieldDimensions.yPosLeftGoal;
    const float offsetRatio = std::min(maxLateralDistance / goalHalfWidth, 1.0f);
    
    // 计算最大可用偏移（不超过球门范围）
    const Angle safeMaxOffset = std::min(
      maxAvoidanceAngle * offsetRatio, 
      std::min(baseAngle - rightAngle, leftAngle - baseAngle)
    );
  
    // 增强版方向判断：总是向守门员位置的反方向偏移
    const Angle offset = (maxLateralDistance > 0) ? -safeMaxOffset : safeMaxOffset;
    
    return baseAngle + offset;
  }
};

MAKE_CARD(CodeReleaseKickAtGoalCard);

```

