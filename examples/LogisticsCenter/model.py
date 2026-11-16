"""Logistics Center experiment using the SOCS DEVS engine."""

from argparse import ArgumentParser
from collections import Counter
from math import inf, isfinite

from socs.devs import CoupledDEVS, DEVSph
from socs.devs.DEVSPetriNet import DEVSPetriNet
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginBase

from .motion import agv_plan, crane_plan, traffic_departure

PORTS = {"L1": (2.4, -1.5), "L2": (0.5, -1.5),
         "R1": (14.5, -1.5), "R2": (12.6, -1.5)}
FORK_OFFSETS = (-0.5, 0.5)
CART_OFFSETS = (1.325, 3.125, 4.925)
AGV_LANE_Y = -3.7
DOCKS = {
    (port, slot): (x - offset, AGV_LANE_Y)
    for port, (x, _) in PORTS.items()
    for slot, offset in enumerate(CART_OFFSETS)
}
STATION = (7.5, -9.2)
PARKING = {"agv0": (4.0, -11.4), "agv1": (12.0, -11.4),
           "agv2": (-4.0, -11.4), "agv3": (20.0, -11.4)}
BAY_ENTRIES = {name: (x - 3, -9.2) for name, (x, _) in PARKING.items()}
BAY_EXITS = {name: (x + 2, -9.2) for name, (x, _) in PARKING.items()}
RECEIVING = (0.5, -9.2)
BUFFER = 2
BIN_CAPACITY = 12
FLOWS = ("out", "in", "out_return", "all")

ROUTE = ((-8.5, -9.2),
         *sorted((RECEIVING, STATION, *BAY_ENTRIES.values(), *BAY_EXITS.values())),
         (23.5, -9.2), (23.5, AGV_LANE_Y),
         *sorted(set(DOCKS.values()), reverse=True), (-8.5, AGV_LANE_Y))
HORIZON = 1200
SKU_TYPES = "ABC"
ORDER_INTERVAL = 15
RACK_CAPACITY = 90
DEFAULT_REORDER_POINT = 12
DEFAULT_LEAD_TIME = 60


def inventory_breakdown(state):
    """Report physical rack stock, reservations, and picked goods awaiting shipment."""
    rack = Counter()
    available = Counter()
    reserved = Counter()
    picked = Counter()
    for item in state["bins"].values():
        sku = item["sku"]
        reservation = len(item["picks"])
        reserved[sku] += reservation
        if item["where"] == "rack":
            rack[sku] += item["stock"]
            available[sku] += item["stock"] - reservation
    for order in state["orders"].values():
        if order["completed"] is None:
            picked.update(Counter(order["requested"]) - Counter(order["remaining"]))
    return {
        sku: dict(rack=rack[sku], available=available[sku], reserved=reserved[sku],
                  picked=picked[sku], putaway=state.get("received_boxes", {}).get(sku, 0))
        for sku in SKU_TYPES
    }


def inventory_totals(state):
    """Return available rack stock and total allocation per SKU."""
    return {
        sku: (item["available"], item["reserved"] + item["picked"])
        for sku, item in inventory_breakdown(state).items()
    }


def route_leg(start, end):
    path = [start]
    if start in PARKING.values():
        vehicle = next(name for name, bay in PARKING.items() if bay == start)
        path.extend(((start[0] + 0.6, start[1]), BAY_EXITS[vehicle]))
    target = end
    if end in PARKING.values():
        vehicle = next(name for name, bay in PARKING.items() if bay == end)
        target = BAY_ENTRIES[vehicle]
    index = ROUTE.index(path[-1])
    while path[-1] != target:
        index = (index + 1) % len(ROUTE)
        path.append(ROUTE[index])
    if target != end:
        path.extend(((end[0] - 0.6, end[1]), end))
    return path


class Equipment(DEVSph):
    def __init__(self, name):
        super().__init__(name, variables={"job": None})
        self.add_input_ports("?job")
        self.add_output_ports("!done")
        self.add_phase("Idle", inf, initial=True)
        self.add_phase("Working", lambda model: model._variable.job["duration"])
        self.add_external_transition(
            "Idle", "Working", input_port="?job", reschedule=True,
            action=lambda model, event: setattr(model._variable, "job", event.value),
        )
        self.add_internal_transition(
            "Working", "Idle", output=lambda model: PortValue("!done", model._variable.job),
        )


class Controller(DEVSPetriNet):
    def __init__(self, forks, trailers, rule, demand, flow, reorder_point, lead_time, agvs):
        super().__init__("Dispatch")
        self.forks, self.trailers, self.rule = forks, trailers, rule
        self.flow = flow
        self.reorder_point, self.lead_time = reorder_point, lead_time
        self.add_input_ports("?done")
        self.vehicles = tuple(PARKING)[:agvs]
        self.add_output_ports("!crane", "!pick", *("!" + name for name in self.vehicles))
        bins = {
            i: {"sku": sku, "stock": BIN_CAPACITY, "cell": (i * 7 % 15, i % 6),
                "where": "rack", "picks": []}
            for i, sku in enumerate("ABCABCABC")
        }
        state = dict(bins=bins, pending=list(demand), orders={}, busy={}, replenishment={}, pull_count=0,
                     received_boxes=dict.fromkeys(SKU_TYPES, 0),
                     ports=dict.fromkeys(PORTS), out=[], incoming=[],
                     wake=0, trips=0)
        self.add_place("State", [state])
        self.add_place("Done")
        self.connect_input_port("?done", "Done")
        self.add_transition("Complete")
        self.add_input_arc("Done", "Complete", "job")



    @property
    def state(self):
        return self.tokens("State")[0]

    def tau(self):
        if self.tokens("Done"):
            return 0
        state = self.state
        arrival = state["pending"][0][0] if state["pending"] else inf
        return max(0, min(arrival, state["wake"]) - self.tNow)

    def delta_x(self, event):
        self.receive_tokens(event)
        return True

    def delta_y(self):
        state = self.state
        now = self.tNow
        output = []
        state["wake"] = inf
        while self.tokens("Done"):
            self.finish(state, self.fire("Complete")["job"], now)


        while state["pending"] and state["pending"][0][0] <= now:
            arrival, key, skus, deadline = state["pending"].pop(0)
            if deadline is None:
                self.receive_stock(state, skus)
            else:
                state["orders"][key] = dict(
                    arrival=arrival, due=deadline, requested=skus, remaining=list(skus),
                    unassigned=list(skus), completed=None,
                )

        orders = sorted(state["orders"].items(), key=lambda pair: pair[1]["due"])
        if self.rule == "fifo":
            orders = list(state["orders"].items())
        for key, order in orders:
            for sku in order["unassigned"][:]:
                candidates = [
                    (i, b) for i, b in state["bins"].items()
                    if b["sku"] == sku and b["where"] in {"rack", "retrieving", "outbound", "incoming"}
                    and b["stock"] > len(b["picks"])
                ]
                if candidates:
                    _, bin_data = min(candidates, key=lambda pair: not bool(pair[1]["picks"]))
                    bin_data["picks"].append(key)
                    order["unassigned"].remove(sku)

        if "pick" not in state["busy"] and state["incoming"] and len(state["out"]) < BUFFER:
            i = min(state["incoming"], key=lambda i: self.priority(state, i))
            state["incoming"].remove(i)
            self.start(state, output, "pick", "pick", [i], 1.5 + len(state["bins"][i]["picks"]), now)

        self.pull_stock(state, now)
        self.dispatch_crane(state, output, now)
        for vehicle in self.vehicles:
            if vehicle not in state["busy"]:
                self.dispatch_agv(state, output, vehicle, now)


        self.set_tokens("State", [state])
        return output

    def priority(self, state, i):
        if self.rule == "fifo":
            return 0
        return min((state["orders"][key]["due"] for key in state["bins"][i]["picks"]),
                   default=inf)

    def pull_stock(self, state, now):
        if self.flow not in {"in", "all"}:
            return
        active = [b for b in state["bins"].values() if b["where"] != "shipped"]
        available = Counter()
        for item in active:
            if item.get("flow_mark") != "IN":
                available[item["sku"]] += item["stock"] - len(item["picks"])
        outstanding = state["replenishment"]
        capacity = RACK_CAPACITY - len(active) - sum(eta is not None for eta in outstanding.values())
        for sku in SKU_TYPES:
            if capacity <= 0:
                break
            if sku in outstanding or available[sku] > self.reorder_point:
                continue
            state["pull_count"] += 1
            eta = now + self.lead_time
            outstanding[sku] = eta
            state["pending"].append((eta, f"P{state['pull_count']}", sku, None))
            capacity -= 1
        state["pending"].sort()

    def receive_stock(self, state, skus):
        for sku in skus:
            bins = state["bins"]
            occupied = {b["cell"] for b in bins.values() if b["where"] != "shipped"}
            cell = next((x, z) for z in range(6) for x in range(15)
                        if (x, z) not in occupied)
            bin_id = next(i for i in range(RACK_CAPACITY)
                          if i not in bins or bins[i]["where"] == "shipped")
            bins[bin_id] = dict(
                sku=sku, stock=BIN_CAPACITY, cell=cell, where="receiving", picks=[], flow_mark="IN",
            )
            state["replenishment"][sku] = None

    def start(self, state, output, machine, action, bins, duration, now, **details):
        job = dict(machine=machine, action=action, bins=bins,
                   start=now, duration=duration, **details)
        state["busy"][machine] = job
        output.append(PortValue("!" + machine, job))

    def dispatch_crane(self, state, output, now):
        if "crane" in state["busy"]:
            return
        ports = state["ports"]
        bins = state["bins"]
        ready = [(port, entry[1]) for port, entry in ports.items()
                 if entry and entry[0] == "putaway"]
        free = [port for port, entry in ports.items() if entry is None]
        requests = [(i, b) for i, b in bins.items() if b["where"] == "rack" and b["picks"]]
        if self.rule != "fifo":
            requests.sort(key=lambda pair: min(state["orders"][key]["due"] for key in pair[1]["picks"]))
        if ready and (len(state["out"]) == BUFFER or not requests or not free):
            stops = ready[:self.forks]
            action = "store"
        else:
            action = "retrieve"
            committed = sum(b["where"] in {"retrieving", "outbound", "incoming"}
                            for b in bins.values())
            committed += sum(len(j["bins"]) for j in state["busy"].values()
                             if j["action"] == "deliver")
            limit = max(0, min(self.forks, BUFFER - committed))
            if limit == 2:
                pair = next(([side + "1", side + "2"] for side in ("L", "R")
                             if side + "1" in free and side + "2" in free), None)
                if pair:
                    free = pair
            stops = list(zip(free, (i for i, _ in requests)))[:limit]
            if not stops and ready:
                stops = ready[:self.forks]
                action = "store"
        if not stops:
            return
        if action == "retrieve":
            for port, i in stops:
                ports[port] = ("reserved_out", i)
                bins[i]["where"] = "retrieving"
                bins[i]["flow_mark"] = "OUT"
        points = [(7.5, 0)]
        rack_points = [(bins[i]["cell"][0] + 0.5 - FORK_OFFSETS[slot],
                        bins[i]["cell"][1] * 0.75)
                       for slot, (_, i) in enumerate(stops)]
        port_points = [(PORTS[port][0] - FORK_OFFSETS[slot], 0)
                       for slot, (port, _) in enumerate(stops)]
        points += rack_points + port_points if action == "retrieve" else port_points + rack_points
        points.append((7.5, 0))
        motion, duration = crane_plan(points, action, len(stops))
        self.start(state, output, "crane", action, [i for _, i in stops], duration, now,
                   stops=stops, points=points, motion=motion)

    def dispatch_agv(self, state, output, vehicle, now):
        ports = state["ports"]
        free = [p for p, entry in ports.items() if entry is None]
        reserved = sum(len(job["bins"]) for job in state["busy"].values()
                       if job["action"] == "deliver")
        slots = BUFFER - len(state["incoming"]) - reserved
        ready = [(p, entry[1]) for p, entry in ports.items() if entry and entry[0] == "pickup"]
        ready.sort(key=lambda stop: self.priority(state, stop[1]))
        ready = ready[:min(self.trailers, slots)]
        receiving = [i for i, b in state["bins"].items() if b["where"] == "receiving"]
        if free and state["out"] and (len(state["out"]) == BUFFER or not ready):
            action, source = "return", STATION
            stops = list(zip(free, state["out"]))[:self.trailers]
            for _, i in stops:
                state["out"].remove(i)
        elif ready:
            if self.rule == "slack" and len(ready) < min(self.trailers, slots):
                deadline = min(state["orders"][key]["due"] for _, i in ready
                               for key in state["bins"][i]["picks"])
                crane = state["busy"].get("crane")
                if crane and crane["action"] == "retrieve":
                    eta = crane["start"] + crane["duration"]
                    if now < eta <= min(now + 4, deadline - 18):
                        state["wake"] = min(state["wake"], eta)
                        return
            action, source, stops = "deliver", STATION, ready
        elif free and receiving:
            action, source = "inbound", RECEIVING
            stops = list(zip(free, receiving))[:self.trailers]
        else:
            return

        stops.sort(key=lambda stop: PORTS[stop[0]][0], reverse=True)
        parking = PARKING[vehicle]
        anchors = [parking]
        if action != "deliver":
            anchors.append(source)
        port_indices = list(range(len(anchors), len(anchors) + len(stops)))
        anchors.extend(DOCKS[port, slot] for slot, (port, _) in enumerate(stops))
        drop_index = len(anchors) if action == "deliver" else None
        if drop_index is not None:
            anchors.append(STATION)
        anchors.append(parking)
        legs = [route_leg(a, b) for a, b in zip(anchors, anchors[1:])]
        handoffs = set(port_indices) | {drop_index if action == "deliver" else 1}
        points, motion, duration = agv_plan(legs, 1.8 - 0.15 * self.trailers, handoffs)
        booked = [state["busy"][name] for name in self.vehicles if name in state["busy"]]
        candidate = dict(points=points, motion=motion, start=0, depart=0, duration=duration)
        start = traffic_departure(now, candidate, booked, (0, *CART_OFFSETS[:self.trailers]))
        state["trips"] += 1
        for port, i in stops:
            ports[port] = ("in_transit", i)
            state["bins"][i]["where"] = "transport"
        self.start(state, output, vehicle, action, [i for _, i in stops],
                   start + duration - now, now, stops=stops, points=points, depart=start, motion=motion,
                   port_indices=port_indices, drop_index=drop_index)

    def finish(self, state, job, now):
        action = job["action"]
        bins = state["bins"]
        del state["busy"][job["machine"]]
        if action == "pick":
            i, = job["bins"]
            for key in bins[i]["picks"]:
                order = state["orders"][key]
                order["remaining"].remove(bins[i]["sku"])
                if not order["remaining"]:
                    order["completed"] = now
            bins[i]["stock"] -= len(bins[i]["picks"])
            bins[i]["picks"].clear()
            if self.flow == "out" or bins[i]["stock"] == 0:
                bins[i]["where"] = "shipped"
            else:
                bins[i]["where"] = "out"
                bins[i]["flow_mark"] = "RETURN"
                state["out"].append(i)
            return
        for port, i in job["stops"]:
            place, entry = {
                "retrieve": ("outbound", ("pickup", i)),
                "deliver": ("incoming", None),
                "return": ("port", ("putaway", i)),
                "inbound": ("port", ("putaway", i)),
                "store": ("rack", None),
            }[action]
            bins[i]["where"] = place
            if action == "store":
                if bins[i].pop("flow_mark", None) == "IN":
                    sku = bins[i]["sku"]
                    state["received_boxes"][sku] += bins[i]["stock"]
                    state["replenishment"].pop(sku, None)
            state["ports"][port] = entry
            if action == "deliver":
                state["incoming"].append(i)


def demand():
    initial = [
        (0, "O1", "AB", 150),
        (3, "U1", "BC", 105),
        (15, "O2", "ABC", 180),
        (23, "U2", "AC", 150),
        (28, "O3", "B", 190),
        (36, "U3", "AB", 170),
        (54, "U4", "ABC", 185),
    ]
    ongoing = [
        (when, f"G{index}", SKU_TYPES, when + 180)
        for index, when in enumerate(range(60, HORIZON, ORDER_INTERVAL), 1)
    ]
    return sorted(initial + ongoing)


def make_model(forks=1, trailers=1, rule="slack", flow="all",
               reorder_point=DEFAULT_REORDER_POINT, lead_time=DEFAULT_LEAD_TIME, agvs=2):
    if not isinstance(reorder_point, int) or reorder_point < 0:
        raise ValueError("Reorder point must be a nonnegative integer.")
    if not isfinite(lead_time) or lead_time <= 0:
        raise ValueError("Lead time must be finite and positive.")
    if agvs not in range(1, len(PARKING) + 1):
        raise ValueError("AGV count must be from 1 through 4.")
    if flow not in FLOWS:
        raise ValueError(f"Unknown flow: {flow}")
    model = CoupledDEVS("LogisticsCenter")
    arrivals = [] if flow == "in" else demand()
    control = Controller(forks, trailers, rule, arrivals, flow, reorder_point, lead_time, agvs)
    model.add_child(control)
    for name in ("crane", *control.vehicles, "pick"):
        equipment = Equipment(name)
        model.add_child(equipment)
        model.add_couple((control, "!" + name), (equipment, "?job"))
        model.add_couple((equipment, "!done"), (control, "?done"))
    return model, control


def simulate(forks, trailers, rule, flow="all",
             reorder_point=DEFAULT_REORDER_POINT, lead_time=DEFAULT_LEAD_TIME, agvs=2):
    model, control = make_model(forks, trailers, rule, flow, reorder_point, lead_time, agvs)
    engine = TESEnginBase(model, TimeSimulationEnd=HORIZON)
    engine.bVerbose = False
    while model.tNext < HORIZON or engine.XBuffer:
        engine.execute_one_step()
    return control.state


def summarize(state):
    urgent = [o for key, o in state["orders"].items() if key.startswith("U")]
    times = [o["completed"] - o["arrival"] for o in urgent if o["completed"] is not None]
    timely = sum(o["completed"] is not None and o["completed"] <= o["due"] for o in urgent)
    regular = [o for key, o in state["orders"].items() if key.startswith("O")]
    last = max((o["completed"] for o in regular if o["completed"] is not None), default=None)
    return (f"urgent on time {timely}/{len(urgent)}  "
            f"urgent max lead time {max(times, default=None)}  "
            f"last regular completion {last}  AGV trips {state['trips']}  pull orders {state['pull_count']}")


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--forks", type=int, choices=(1, 2), default=2)
    parser.add_argument("--agvs", type=int, choices=range(1, 5), default=2)
    parser.add_argument("--trailers", type=int, choices=(1, 2, 3), default=2)
    parser.add_argument("--rule", choices=("fifo", "immediate", "slack"), default="slack")
    parser.add_argument("--flow", choices=FLOWS, default="all",
                        help="Select outbound, inbound, outbound with returns, or all flows.")
    parser.add_argument("--reorder-point", type=int, default=DEFAULT_REORDER_POINT,
                        help="Pull one Bin at or below this number of unreserved boxes per SKU.")
    parser.add_argument("--lead-time", type=float, default=DEFAULT_LEAD_TIME,
                        help="Seconds from pull order to external receiving.")
    parser.add_argument("--sweep", action="store_true")
    parser.add_argument("--view", action="store_true")
    args = parser.parse_args()
    if args.reorder_point < 0:
        parser.error("--reorder-point must be nonnegative")
    if not isfinite(args.lead_time) or args.lead_time <= 0:
        parser.error("--lead-time must be finite and positive")
    settings = dict(reorder_point=args.reorder_point, lead_time=args.lead_time, agvs=args.agvs)
    if args.view:
        from .viewer import run
        run(args.forks, args.trailers, args.rule, args.flow, **settings)
    elif args.sweep:
        for forks in (1, 2):
            for trailers in (1, 2, 3):
                for rule in ("fifo", "immediate", "slack"):
                    print(f"{forks} fork / {trailers} trailers / {rule}: "
                          + summarize(simulate(forks, trailers, rule, args.flow, **settings)))
    else:
        print(summarize(simulate(args.forks, args.trailers, args.rule, args.flow, **settings)))


if __name__ == "__main__":
    main()



