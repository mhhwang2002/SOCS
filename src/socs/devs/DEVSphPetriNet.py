"""Phase graph support for colored Petri net models."""

from copy import deepcopy

from .DEVSph import DEVSph
from .DEVSPetriNet import _PetriNet


class DEVSphPetriNet(_PetriNet, DEVSph):
    def __init__(self, name, parent=None, *, variables=None):
        super().__init__(name, parent, variables=variables)
        self._init_petri_net()

    def set_initial_state(self):
        super().set_initial_state()
        self._marking = deepcopy(self._initial_marking)

    def get_state(self):
        return super().get_state(), deepcopy(self._marking)

    def set_state(self, state):
        phase_state, marking = state
        super().set_state(phase_state)
        self._marking = deepcopy(marking)
