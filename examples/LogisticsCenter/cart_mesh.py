"""Polygon bodywork for the Logistics Center towing vehicles and carts."""

from math import cos, pi, sin


def loft(name, rings, color):
    count = len(rings[0])
    vertices = [vertex for ring in rings for vertex in ring]
    triangles = [(0, i + 1, i) for i in range(1, count - 1)]
    for layer in range(len(rings) - 1):
        base = layer * count
        for i in range(count):
            j = (i + 1) % count
            triangles.extend(((base + i, base + j, base + count + j),
                              (base + i, base + count + j, base + count + i)))
    top = len(vertices) - count
    triangles.extend((top, top + i, top + i + 1) for i in range(1, count - 1))
    return dict(name=name, vertices=vertices, triangles=triangles, color=color)


def deck_ring(length, width, z):
    cut = min(0.09, length / 2, width / 2)
    return [(length - cut, -width, z), (length, -width + cut, z),
            (length, width - cut, z), (length - cut, width, z),
            (-length + cut, width, z), (-length, width - cut, z),
            (-length, -width + cut, z), (-length + cut, -width, z)]


def cart_meshes():
    yield loft("Beveled chassis",
               [deck_ring(0.58, 0.37, 0.19), deck_ring(0.65, 0.43, 0.25),
                deck_ring(0.65, 0.43, 0.38), deck_ring(0.59, 0.37, 0.44)],
               (0.12, 0.25, 0.32, 1))
    yield loft("Transfer deck", [deck_ring(0.56, 0.35, 0.44), deck_ring(0.55, 0.34, 0.465)],
               (0.63, 0.70, 0.73, 1))
    yield from wheel_meshes()


def wheel_meshes():
    for x in (-0.41, 0.41):
        for side in (-1, 1):
            rings = [
                [(x + radius * cos(2 * pi * i / 16), side * 0.37 + lateral,
                  0.14 - radius * sin(2 * pi * i / 16)) for i in range(16)]
                for lateral, radius in ((-0.066, 0.106), (-0.041, 0.136),
                                        (0.041, 0.136), (0.066, 0.106))
            ]
            yield loft("Beveled wheel", rings, (0.09, 0.10, 0.11, 1))


def body_ring(center_x, length, width, z, center_y=0):
    return [(x + center_x, y + center_y, height) for x, y, height in deck_ring(length, width, z)]


def tugger_meshes():
    amber = (0.97, 0.48, 0.08, 1)
    charcoal = (0.10, 0.13, 0.16, 1)
    glass = (0.14, 0.26, 0.34, 1)
    yield from wheel_meshes()
    yield loft("Lower bumper and sills",
               [deck_ring(0.60, 0.40, 0.17), deck_ring(0.68, 0.46, 0.23),
                deck_ring(0.68, 0.46, 0.32), deck_ring(0.64, 0.45, 0.36)], charcoal)
    yield loft("Rounded body shoulders",
               [deck_ring(0.63, 0.43, 0.30), deck_ring(0.66, 0.46, 0.37),
                deck_ring(0.60, 0.40, 0.47)], amber)
    yield loft("Sloped front bonnet",
               [body_ring(0.29, 0.33, 0.37, 0.44), body_ring(0.26, 0.30, 0.34, 0.50),
                body_ring(0.17, 0.22, 0.30, 0.58)], amber)
    yield loft("Raked cab glazing",
               [body_ring(-0.18, 0.33, 0.34, 0.47), body_ring(-0.23, 0.24, 0.28, 0.79)],
               glass)
    yield loft("Cab roof",
               [body_ring(-0.23, 0.25, 0.29, 0.78), body_ring(-0.23, 0.23, 0.27, 0.84)],
               amber)
    yield loft("Front grille",
               [body_ring(0.638, 0.016, 0.17, 0.34), body_ring(0.625, 0.016, 0.15, 0.44)],
               charcoal)
    for side in (-1, 1):
        yield loft("Headlight",
                   [body_ring(0.623, 0.025, 0.070, 0.36, side * 0.27),
                    body_ring(0.606, 0.030, 0.063, 0.43, side * 0.27)],
                   (0.94, 0.97, 1.0, 1))
        yield loft("Tail light",
                   [body_ring(-0.632, 0.019, 0.06, 0.34, side * 0.27),
                    body_ring(-0.622, 0.019, 0.05, 0.41, side * 0.27)],
                   (0.81, 0.10, 0.08, 1))
