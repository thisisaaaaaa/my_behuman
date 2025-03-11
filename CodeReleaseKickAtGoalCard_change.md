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
 #include "Representations/Modeling/ObstacleModel.h"
 
 
 
 CARD(CodeReleaseKickAtGoalCard,
 {,
   CALLS(Activity),
   CALLS(InWalkKick),
   CALLS(LookForward),
   CALLS(Stand),
   CALLS(WalkAtRelativeSpeed),
   CALLS(WalkToTarget),
   CALLS(Say),
   REQUIRES(FieldBall),
   REQUIRES(FieldDimensions),
   REQUIRES(RobotPose),
   REQUIRES(ObstacleModel),
   DEFINES_PARAMETERS(
   {,
     (float)(0.3f) walkSpeed,//更改速度
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
     
     state(done){
      action
      {
        theStandSkill();//一次踢球机会
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
         if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
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
         if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
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
       const Angle angleToGoal = calcAngleToGoal1();
 
       transition
       {
         if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
           goto start;
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
     // 获取对方球门中心的全局坐标
     const Vector2f goalCenter(theFieldDimensions.xPosOpponentGroundline, 0.f);
     // 转换到机器人坐标系
     const Vector2f goalRelative = theRobotPose.inversePose * goalCenter;
         
     // 计算机器人面向球门中心的角度
     Angle angleToGoal = goalRelative.angle();
     
     return angleToGoal; // 默认返回原始射门角度
   }
   Angle calcAngleToGoal1() const
   {
     // 获取对方球门中心的全局坐标
     const Vector2f goalCenter(theFieldDimensions.xPosOpponentGroundline, 0.f);
     // 转换到机器人坐标系
     const Vector2f goalRelative = theRobotPose.inversePose * goalCenter;
         
     // 计算机器人面向球门中心的角度
     Angle angleToGoal = goalRelative.angle();
     // 获取唯一的对手位置（假设只有一个障碍物）
     Vector2f opponentPosition;
     bool hasOpponent = false;
     for (const auto& obstacle : theObstacleModel.obstacles)
     {
      theSaySkill("is valid!!!");
       if (obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent)
       {
         opponentPosition = theRobotPose.inversePose * obstacle.center;
         hasOpponent = true;
         theSaySkill("I have seen the opponent");
         break;  // 只处理一个障碍物
       }
     }
     // 如果对手挡在射门路径上，则调整角度
     if (hasOpponent)
     {
       Angle obstacleAngle = opponentPosition.angle();
       if (obstacleAngle - angleToGoal < 5_deg || angleToGoal - obstacleAngle < 5_deg) // 如果对手位于射门角度 ±10° 以内
       {
        theSaySkill("sir,I have done somthing !");
         if (opponentPosition.y() > 0)
           return angleToGoal - 30_deg;  // 向右避开
         else
           return angleToGoal + 30_deg;  // 向左避开
       }
     }
     return angleToGoal; // 默认返回原始射门角度
   }
 };
 
 
 
 
 MAKE_CARD(CodeReleaseKickAtGoalCard);
 