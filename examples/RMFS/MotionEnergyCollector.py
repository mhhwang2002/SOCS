"""Passive DEVS observer of straight motion plans and command arrivals."""

from math import inf

from socs.devs import DEVSph


class MotionEnergyCollector(DEVSph):
    def __init__(self, names, *, track_load=False):
        variables = {'events': {name: [] for name in names}}
        if track_load:
            variables['load'] = {name: [(0., False)] for name in names}
        super().__init__('MotionEnergyCollector', variables=variables)
        self.add_phase('Collect', inf, initial=True)
        for name in names:
            self.add_input_ports(f'?plan_{name}', f'?arrival_{name}')
            self.add_external_transition(
                'Collect', 'Collect', input_port=f'?plan_{name}', reschedule=False,
                action=lambda model, event, name=name: model.record_plan(name, event.value))
            self.add_external_transition(
                'Collect', 'Collect', input_port=f'?arrival_{name}', reschedule=False,
                action=lambda model, event, name=name: model.record_arrival(name, event.value))

            if track_load:
                for event, loaded in (('lifted', True), ('lowered', False)):
                    port = f'?{event}_{name}'
                    self.add_input_ports(port)
                    self.add_external_transition(
                        'Collect', 'Collect', input_port=port, reschedule=False,
                        action=lambda model, event, name=name, loaded=loaded:
                            model._variable.load[name].append((model.tNow, loaded)))

    def load_events(self, name):
        return tuple(self._variable.load[name])

    def record_plan(self, name, plan):
        self._variable.events[name].append((self.tNow, plan['start_time'], plan['profile']))

    def record_arrival(self, name, position):
        self._variable.events[name].append((self.tNow, tuple(position), None))

    def motion_intervals(self, name):
        """Yield (effective start, effective end, profile origin time, profile)."""
        events = self._variable.events[name]
        for (start, origin, profile), (end, _, _) in zip(events, events[1:]):
            if profile is not None and end > start:
                yield start, end, origin, profile

    def arrival_time(self, name, goal):
        return next(time for time, position, profile in reversed(self._variable.events[name])
                    if profile is None and position == goal)


def attach_motion_collector(model, robots, *, track_load=False):
    collector = MotionEnergyCollector(tuple(robot.name for robot in robots), track_load=track_load)
    model.add_components(collector)
    for robot in robots:
        model.add_couples(
            ((robot, '!motion_plan'), (collector, f'?plan_{robot.name}')),
            ((robot, '!arrived'), (collector, f'?arrival_{robot.name}')),
        )
        if track_load:
            for event in ('lifted', 'lowered'):
                model.add_couples(((robot, f'!{event}'), (collector, f'?{event}_{robot.name}')))
    return collector
