"""Construct fixed experimental schedules and hold an exit blocker in DEVS."""

from math import inf

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

from ..path_geometry import TuplePath
from ..wait_duplication import SearchResult


def serial_legs_witness(solver, legs):
    state = [0] * len(solver.names)
    states, rows = [tuple(state)], []
    for robot, target in legs:
        while state[robot] < target:
            row, _ = solver._choices(state, (robot,))
            rows.append(row)
            state[robot] += 1
            states.append(tuple(state))
    return SearchResult('feasible', TuplePath(solver.names, tuple(rows)), tuple(states),
                        len(states), len(rows), 0., 'Constructed and validated by the model builder.')



class BlockerDeparture(DEVSph):
    """Hold the blocker at the same starting position until the scripted release time."""

    def __init__(self, release_time, name='BlockerDeparture'):
        super().__init__(name, variables={'command': None})
        self.add_input_ports('?command')
        self.add_output_ports('!command')
        self.add_phase('Idle', inf, initial=True)
        self.add_phase('Holding', lambda m: max(0., release_time - m.tNow))
        self.add_external_transition('Idle', 'Holding', input_port='?command', reschedule=True,
                                     action=lambda m, e: setattr(m._variable, 'command', e.value))
        self.add_internal_transition('Holding', 'Idle',
                                     output=lambda m: PortValue('!command', m._variable.command))


def hold_departure(model, robot, release_time):
    gate = BlockerDeparture(release_time, f'BlockerDeparture_{robot.name}')
    model.add_components(gate)
    if not hasattr(model, 'departure_holds'):
        model.departure_holds = {}
    model.departure_holds[robot.name] = gate
    source = (model.controller, f'!{robot.name}')
    model.Z[source].remove((robot, '?command'))
    model.add_couples((source, (gate, '?command')), ((gate, '!command'), (robot, '?command')))
