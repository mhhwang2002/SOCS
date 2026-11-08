# ASRS warehouse concept model

This folder contains a conceptual 3D automated storage and retrieval system (AS/RS) based on the sketch in `examples/ASRS.pptx`. The original sketch file is preserved. `ASRS_detailed.pptx` contains the sketch plus two editable detail slides.

## Dimensions

Dimensions read from the sketch:

- 15 bays at 1.0 m each: rack length 15.0 m
- 6 storage levels at 0.75 m pitch: rack height 4.5 m
- 50 mm beam thickness

Dimensions assumed where the sketch did not specify them:

- Rack depth: 1.0 m
- Aisle width: 2.0 m
- Concept warehouse floor: 18 m × 6 m
- Rack post section: 70 mm
- Fork reach: 1.25 m
- Load clearance for lifting: 0.12 m

These assumptions are documented in the Blender scene as well. They are for visualization and kinematics demonstration, not structural or safety engineering.

## Crane and animation

The Blender scene has a keyframed motion demonstration. The DEVS example models simultaneous X rail travel and Z hoist travel, followed by Y-axis fork reach. At the cell, the crane sends a retrieve request; the selected cell sends a box output, clears its occupied state, and the crane attaches the box to its fork. The crane lifts, withdraws, and returns X/Z to its home pose, then approaches the unload station, sends the box out, and lowers the fork. The simulation pauses after Fork Down with the viewer open. Each axis uses acceleration, optional cruise, and deceleration; short moves can omit the cruise phase. The model contains 90 passive `ASRSCell` atomic models and one active `Crane`, joined by request and box-transfer couplings in the `ASRS` coupled model.

The crane home is at the center of cell coordinate `(-1, 0)`, outside the rack (`x=-0.5 m`); cell `(0, 0)` starts at `x=0`. The unload point is at the home X position and 0.5 m of fork reach toward the rack, on a short platform at the lowest transfer height. The floor rails extend to support this outside-rack home position.

Open `ASRS_warehouse.blend` in Blender and play the timeline with Space. The scene is built by `build_asrs.py`.

To launch the coupled simulation and live Panda3D view from the project root:

```powershell
python -m pip install -e ".[visual3d]"
python -m examples.ASRS.run_asrs
```

In the viewer, **Space** starts or continues interactively, **A** runs continuously, **I** pauses at DEVS phase changes, **R** resets the camera, and **Esc** closes the window. Rack uprights, beams, shelf decks, floor rails, stored loads, and the crane are modeled as shaded 3D geometry.

## ACO + SOCS order scheduling experiment

Generate and save a readable JSON problem instance from the project root. The saved file contains the rack cells, product colors, order membership, requested quantity, and shortage quantity:

```powershell
python -m examples.ASRS.generate_aco_instance --seed 42 --output examples/ASRS/instances/demo.json
```

Both optimization and animation load the fixed problem file; `--seed` controls the ACO search and does not change the saved problem. Run the combined objective with:

```powershell
python -m examples.ASRS.aco_optimization --instance examples/ASRS/instances/demo.json --seed 42 --ants 12 --iterations 25
```

The generated example places 50 products (10 of each of five colors) in random cells of a 15 × 6 rack. Ten orders specify color-specific SKU demand. Each order receives five baseline product units; three to five orders request one or two additional units whose colors are also stored in the JSON. ACO can assign a shelf product only to an order that requires that color, so orders compete for shared SKU inventory. The prior `product_ids` are the generator’s baseline allocation; the optimizer may reassign those products among orders.

The single minimized score prioritizes complete orders, then missing product units, then the selected time measure:

`J = M × (number of incomplete orders) + λ × (unshipped product units) + time`

`time` is makespan by default or mean order departure with `--time-objective mean`. The completion coefficient `M` is calculated above the maximum possible shortage and travel terms, so one additional complete order always outranks any change in λ-weighted shortage or time. `λ` is measured in simulation time units per missing color-specific product and defaults to 50,000. The ACO records each product-to-order assignment and respects SKU color.

In this generated instance, demand totals exceed inventory by five units across blue and white SKUs. If every available product is shipped, total unshipped units are therefore fixed at five, and `λ × 5` is the same for every allocation. The complete-order term decides how those five shortages are concentrated among orders; makespan breaks ties. Changing λ affects comparisons when candidate schedules leave different quantities unshipped.

For example:

```powershell
python -m examples.ASRS.aco_optimization --instance examples/ASRS/instances/demo.json --penalty-weight 50 --seed 42
python -m examples.ASRS.run_aco_animation --instance examples/ASRS/instances/demo.json --objective makespan --penalty-weight 50 --seed 42
```

Order-specific demand includes color counts, so the optimizer can consolidate compatible products into complete orders. A pure per-unit shortage penalty cannot distinguish allocations that ship the same total number of products; maximizing complete orders provides that distinction. The model does not allow substitution between colors.

This first model intentionally uses a simple travel-time estimate: constant X/Z speeds, simultaneous axis motion, no acceleration/deceleration, no bin return, and fixed pick and station service times. It is an optimization prototype; the current interactive AS/RS model remains the separate one-box visualization demo.

To send an ACO schedule through the AS/RS Controller and animate the colored products, run:

```powershell
python -m examples.ASRS.run_aco_animation --instance examples/ASRS/instances/demo.json --objective makespan --seed 42
```

Use `--objective mean` to use mean order departure as the time tie-breaker while keeping the complete-order and shortage terms. Pass the same `--instance` file to reproduce the exact problem. The HUD shows the combined objective and λ-weighted shortage. Per-order departure messages show actual shipped quantity / requested quantity. After the simulation ends, press **S** to show makespan, shipped/requested totals, complete/partial order counts, and weighted shortage; press **S** again to return to live progress. The DEVS crane retains its acceleration/cruise/deceleration motion profile, so animation timings can differ from the simplified constant-speed ACO score. **Space** resumes, **A** runs automatically, **I** steps through DEVS events, and **Esc** closes.

After unloading, products are displayed on per-order trays in the outbound staging area. Each order keeps the same left-to-right slot while products accumulate in the gray collection row. When the order leaves the system, its products move to the green complete row or orange partial-penalty row. The viewer legend maps slots to order IDs.

## Files

- `build_asrs.py`: Blender scene and animation builder
- `asrs_devs.py`: passive cell, crane, and coupled ASRS models
- `run_asrs.py`: live Panda3D simulation entry point
- `ASRS_warehouse.blend`: editable Blender scene
- `ASRS_warehouse_preview.png`: rendered preview
- `ASRS_detailed.pptx`: original sketch with rack and crane detail slides
