#include "mowgli_behavior/action_nodes.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"

namespace mowgli_behavior
{

void registerAllNodes(BT::BehaviorTreeFactory& factory)
{
  // Condition nodes
  factory.registerNodeType<IsEmergency>("IsEmergency");
  factory.registerNodeType<IsCharging>("IsCharging");
  factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<IsRainDetected>("IsRainDetected");
  factory.registerNodeType<NeedsDocking>("NeedsDocking");
  factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
  factory.registerNodeType<IsCommand>("IsCommand");

  factory.registerNodeType<IsGPSFixed>("IsGPSFixed");
  factory.registerNodeType<ReplanNeeded>("ReplanNeeded");
  factory.registerNodeType<IsBoundaryViolation>("IsBoundaryViolation");
  factory.registerNodeType<IsNewRain>("IsNewRain");

  // Action nodes
  factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StopMoving>("StopMoving");
  factory.registerNodeType<ClearCostmap>("ClearCostmap");
  factory.registerNodeType<PublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<WaitForDuration>("WaitForDuration");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<PlanCoveragePath>("PlanCoveragePath");
  factory.registerNodeType<FollowCoveragePath>("FollowCoveragePath");
  factory.registerNodeType<SaveSlamMap>("SaveSlamMap");
  factory.registerNodeType<BackUp>("BackUp");
  factory.registerNodeType<ClearCommand>("ClearCommand");
  factory.registerNodeType<ReplanCoverage>("ReplanCoverage");
  factory.registerNodeType<SaveObstacles>("SaveObstacles");
  factory.registerNodeType<SetNavMode>("SetNavMode");
  factory.registerNodeType<WasRainingAtStart>("WasRainingAtStart");
  factory.registerNodeType<RecordUndockStart>("RecordUndockStart");
  factory.registerNodeType<CalibrateHeadingFromUndock>("CalibrateHeadingFromUndock");

  // Docking nodes (opennav_docking)
  factory.registerNodeType<DockRobot>("DockRobot");
  factory.registerNodeType<UndockRobot>("UndockRobot");

  // Coverage nodes (opennav_coverage integration)
  factory.registerNodeType<ComputeCoverage>("ComputeCoverage");
  factory.registerNodeType<ExecuteSwathBySwath>("ExecuteSwathBySwath");
  factory.registerNodeType<ExecuteFullCoveragePath>("ExecuteFullCoveragePath");
}

}  // namespace mowgli_behavior
