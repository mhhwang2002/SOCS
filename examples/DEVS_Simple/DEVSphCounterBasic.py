"""Count through explicitly declared phases, one per (phase, count, sum)."""

from math import inf

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

if __package__:
    from .DEVSphCounter import main
else:
    from DEVSphCounter import main


class DEVSphCounterBasic(DEVSph):
    def __init__(self, name, count_no, parent=None):
        super().__init__(name, parent)
        if count_no < 1:
            raise ValueError("count_no must be positive.")
        self.addOutputPorts("!c")
        self.phase_states = {}
        previous = None
        total = 0
        for count in range(count_no + 1):
            done = count == count_no
            kind = "Done" if done else "Count"
            phase = f"{kind}({count}, {total})"
            self.phase_states[phase] = kind, count, total
            self.add_phase(phase, tau=inf if done else 5, initial=count == 0)
            if previous is not None:
                self.add_internal_transition(previous, phase, output=PortValue("!c", total - count))
            previous = phase
            total += count + 1

    def display_state(self):
        return self.phase_states[self.phase]


if __name__ == "__main__":
    main(DEVSphCounterBasic, __doc__)
