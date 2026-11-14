"""Run the AGV Petri net controller in the shared Panda3D viewer."""

from .agv_petri import make_model
from .run_panda import main
from .petri_traffic_map import PetriTrafficMap

if __name__ == "__main__":
    main(make_model, "AGVSystemPetri", PetriTrafficMap)
