# Import some basic libraries and functions
import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import SceneGraphConfig, StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer

from roboContent import model

# Start ModelVisualizer
meshcat = StartMeshcat()

# phaser = Phaser.AddModelFromFile()

# import URDF model
simRobo_url = (
    "./simple_walking_robot.urdf"
    
)
simRobo = model

visualizer = ModelVisualizer(meshcat=meshcat)
# visualizer.parser().AddModels(url=simRobo_url)
visualizer.parser().AddModelsFromString(model,"urdf")
test_mode = True if "TEST_SRCDIR" in os.environ else False

# Start the interactive visualizer.
# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)