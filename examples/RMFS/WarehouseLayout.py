"""The archived RMFS warehouse geometry, independent of archived controllers."""
from heapq import heappop,heappush
import json
from math import dist,inf
from pathlib import Path


def load_layout(*, home_capacity=10):
    data=json.loads(Path(__file__).with_name('warehouse_layout.json').read_text())
    segments=tuple((tuple(a),tuple(b)) for a,b in data['segments'])
    # Keep the original ten-home layout unchanged for the existing experiments.
    # Larger fleets also use side bays; thirty robots require ten extra side spurs.
    if home_capacity > 30:
        raise ValueError('The warehouse fleet supports at most 30 dedicated homes.')
    if home_capacity > 20:
        segments = list(segments)
        for x, outside in ((0., -3.5), (42., 45.5)):
            for y in (-11., -7., -3.5, 4.5, 8.5):
                junction = (x, y)
                for index, (a, b) in enumerate(segments):
                    if a[0] == b[0] == x and min(a[1], b[1]) < y < max(a[1], b[1]):
                        segments[index:index+1] = [(a, junction), (junction, b)]
                        break
                else:
                    raise ValueError(f'No boundary aisle for home junction {junction}')
                segments.append((junction, (outside, y)))
        segments = tuple(segments)
    points=sorted({p for edge in segments for p in edge})
    adjacency={p:[] for p in points}
    for a,b in segments:adjacency[a].append(b);adjacency[b].append(a)
    racks={tuple(r['cell']):tuple(r['position']) for r in data['racks']}
    stations={r['name']:tuple(r['position']) for r in data['workstations']}
    bottom = min(p[1] for p in points)
    homes = sorted((p for p in points if p[1] == bottom and len(adjacency[p]) == 1), key=repr)
    if home_capacity > 10:
        homes += sorted((p for p in points if p[0] in (-3.5,45.5) and len(adjacency[p]) == 1), key=repr)
    return dict(control_points={p:p for p in points},segments=segments,
                adjacency={p:tuple(sorted(v)) for p,v in adjacency.items()},
                extent=max(max(p[k] for p in points)-min(p[k] for p in points) for k in (0,1)),
                warehouse_racks=racks,warehouse_stations=stations,
                warehouse_empty_cells=(),rack_size=(1.5,1.5),warehouse_homes=tuple(homes))


def shortest_route(layout,start,goal,blocked=()):
    """Weighted A* on the given aisle graph; no runtime route optimization."""
    blocked=set(blocked)-{start,goal}
    costs={start:0.};previous={};queue=[(dist(start,goal),0.,start)]
    while queue:
        _,cost,node=heappop(queue)
        if cost!=costs[node]:continue
        if node==goal:
            route=[goal]
            while route[-1]!=start:route.append(previous[route[-1]])
            return tuple(reversed(route))
        for point in layout['adjacency'][node]:
            if point in blocked:continue
            value=cost+dist(node,point)
            if value<costs.get(point,inf):
                costs[point]=value;previous[point]=node
                heappush(queue,(value+dist(point,goal),value,point))
    raise ValueError('No aisle route between the requested endpoints.')
