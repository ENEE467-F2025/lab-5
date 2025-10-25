from dataclasses import dataclass, field, fields
from typing import Dict, Any, List

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


@dataclass
class PlannerParams:
    # Core / misc
    planning_algorithm: str = "rrt_star"
    stop_if_plan_found: bool = True
    verbose: bool = False
    moveit_config_name: str = "robot_3r_moveit_config"
    def_group_state: str = "arm_ready"

    # RRT* specific
    rrts_expand_dist: float = 0.3
    rrts_path_resolution: float = 0.1
    rrts_max_iter: int = 300
    rrts_connect_circle_dist: int = 20
    rrts_search_until_max_iter: bool = False
    rrts_goal_sample_rate: float = 0.3
    use_goal_biased_sampling: bool = False
    goal_noise_sigma: float = 0.05

    # Collision
    min_obs_dist: float = 0.1
    collision_checker: str = "proximity"
    check_collision: bool = True
    use_collision_service: bool = False
    proximity_alert: bool = False
    start_goal_collision: str = ""

    # Viz and metrics
    show_jsp_waypoints: bool = True
    show_ee_path: bool = False
    print_metrics: bool = True

    # Random and failure handling
    random_seed: int = 42
    max_planning_attempts: int = 1
    stop_on_failure: bool = True

    # Goal config default; overridden by SRDF named state if available
    goal_config: List[float] = field(default_factory=lambda: [1.5093, 0.6072, 1.4052])

    # Human-friendly descriptions
    _descriptions: Dict[str, str] = field(default_factory=lambda: {
        "planning_algorithm": "Planning algorithm to use; only 'rrt_star' is supported for now, 'rrt' is planned.",
        "stop_if_plan_found": "Stop planning node if a valid plan is found.",
        "verbose": "Whether to print detailed info during planning.",
        "moveit_config_name": "Name of MoveIt config package.",
        "def_group_state": "Named configuration from the SRDF file.",
        "rrts_expand_dist": "Maximum joint-space expansion distance per step (radians).",
        "rrts_path_resolution": "Sampling resolution along edges (radians).",
        "rrts_max_iter": "Maximum planner iterations.",
        "rrts_connect_circle_dist": "Connection radius factor for RRT* neighbor search.",
        "rrts_search_until_max_iter": "Continue searching until max_iter to improve path.",
        "rrts_goal_sample_rate": "Goal-biased sampling rate (0..1).",
        "use_goal_biased_sampling": "Enable goal-biased sampling.",
        "goal_noise_sigma": "Std dev of Gaussian noise added to goal samples (radians).",
        "min_obs_dist": "Minimum obstacle distance threshold (m).",
        "collision_checker": "Collision checker type: 'bvol' or 'proximity'.",
        "check_collision": "Whether to check for collisions.",
    "use_collision_service": "If true, use the CheckCollision service instead of local FCL.",
        "proximity_alert": "Proximity alert flag.",
        "start_goal_collision": "Whether start or goal is in collision.",
        "show_jsp_waypoints": "Visualize joint-space waypoints.",
        "show_ee_path": "Visualize end-effector path.",
        "print_metrics": "Print planning metrics.",
        "random_seed": "Random seed for reproducibility.",
        "max_planning_attempts": "Max planning attempts before giving up.",
        "stop_on_failure": "Stop node when planning fails.",
        "goal_config": "Goal joint configuration if not using SRDF named state.",
    })

    def declare(self, node: Node) -> None:
        """Declare all parameters on the given node."""
        for f in fields(self):
            name = f.name
            # Skip internal descriptions dict
            if name.startswith("_"):
                continue
            default_value = getattr(self, name)
            desc = self._descriptions.get(name, "")
            try:
                node.declare_parameter(name, value=default_value, descriptor=ParameterDescriptor(description=desc))
            except Exception:
                # Parameter may already be declared; skip
                pass

    def load(self, node: Node) -> None:
        """Populate dataclass fields from node parameters."""
        for f in fields(self):
            name = f.name
            if name.startswith("_"):
                continue
            try:
                param = node.get_parameter(name)
                pval = param.get_parameter_value()
                # Map type to Python value
                if hasattr(pval, "string_value") and pval.string_value != "":
                    value = pval.string_value
                elif hasattr(pval, "bool_value") and (pval.type == 1 or pval.bool_value in (True, False)):
                    value = pval.bool_value
                elif hasattr(pval, "double_value") and (pval.double_value or pval.type == 2):
                    value = pval.double_value
                elif hasattr(pval, "integer_value") and (pval.integer_value or pval.type == 3):
                    value = pval.integer_value
                elif hasattr(pval, "double_array_value") and pval.double_array_value:
                    value = list(pval.double_array_value)
                elif hasattr(pval, "integer_array_value") and pval.integer_array_value:
                    value = list(pval.integer_array_value)
                else:
                    # fallback to the dataclass default
                    value = getattr(self, name)
                setattr(self, name, value)
            except Exception:
                # keep default if not retrievable
                pass


def declare_goal_config(node: Node, group_states: Dict[str, List[float]] | None, def_group_state: str, fallback_goal: List[float]) -> List[float]:
    """Declare 'goal_config' parameter using SRDF named state if available, otherwise fallback list.
    Returns the loaded goal_config list.
    """
    if group_states is not None and def_group_state in group_states:
        default_goal = group_states[def_group_state]
    else:
        default_goal = fallback_goal

    try:
        node.declare_parameter(
            "goal_config",
            value=default_goal,
            descriptor=ParameterDescriptor(description="Goal joint configuration; defaults to named SRDF state if available."),
        )
    except Exception:
        pass

    try:
        return list(node.get_parameter("goal_config").get_parameter_value().double_array_value)
    except Exception:
        return list(default_goal)
