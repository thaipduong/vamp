import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from fire import Fire

# Starting configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

# Starting configuration
a_flat = [0., -0.785, 0., -2.356, 0., 1.571, 0.785, 0., 0., 0., 0., 0., 0., 0.]

# Goal configuration
b_flat = [2.35, 1., 0., -0.8, 0, 2.5, 0.785, 0., 0., 0., 0., 0., 0., 0.]

# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]

#problem = [[0.35, 0.35, 0.8]]
#problem = [[0.35, -0.35, 0.8]]


def main(
    variation: float = 0.01,
    benchmark: bool = False,
    n_trials: int = 100,
    radius: float = 0.2,
    visualize: bool = True,
    planner: str = "flat_prm",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs)

    if benchmark:
        random.seed(0)
        np.random.seed(0)

        results = []
        spheres = [np.array(sphere) for sphere in problem]
        for _ in range(n_trials):
            random.shuffle(spheres)
            spheres_copy = copy.deepcopy(spheres)

            e = vamp.Environment()
            for sphere in spheres_copy:
                sphere += np.random.uniform(low = -variation, high = variation, size = (3, ))
                e.add_sphere(vamp.Sphere(sphere, radius))

            if vamp.panda.validate(a, e) and vamp.panda.validate(b, e):
                result = planner_func(a, b, e, plan_settings)
                simple = vamp_module.simplify(result.path, e, simp_settings)
                results.append(vamp.results_to_dict(result, simple))

        df = pd.DataFrame.from_dict(results)

        # Convert to microseconds
        df["planning_time"] = df["planning_time"].dt.microseconds
        df["simplification_time"] = df["simplification_time"].dt.microseconds

        # Get summary statistics
        stats = df[[
            "planning_time",
            "simplification_time",
            "initial_path_cost",
            "simplified_path_cost",
            "planning_iterations"
            ]].describe()

        print(stats)

    if visualize:
        from vamp import pybullet_interface as vpb

        robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
        sim = vpb.PyBulletSimulator(
            str(robot_dir / f"panda_spherized.urdf"), vamp.ROBOT_JOINTS['panda'], True
            )

        e = vamp.Environment()
        for sphere in problem:
            e.add_sphere(vamp.Sphere(sphere, radius))
            sim.add_sphere(radius, sphere)

        
        path = vamp.panda.traj_to_path(a_flat, b_flat, 3.0, 100)
        result = planner_func(a_flat, b_flat, e, plan_settings)
        prm_path = vamp.panda.flatresult_to_path(result.path, 1.5, 100)
        #simple = vamp_module.simplify(result.path, 
        # e, simp_settings)
        #path = vamp.panda.traj_to_path(result, 100)

        #simple.path.interpolate(vamp.panda.resolution())
        #np_path = path.numpy()
        # free = vamp.panda.validate_traj(a_flat, b_flat, 3.0, e)
        # print("Is the trajectory collision free?: ", free)
        print("Planning time: {} nanoseconds".format(result.nanoseconds))
        sim.animate(prm_path)

if __name__ == "__main__":
    Fire(main)
