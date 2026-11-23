# Completed theory research

As of: 2026-11-19

Date convention: AchievedDate is recorded as 2026-11-19 where the exact completion date is unknown. The assumptions and validation scope stated for each item still apply.

Main document: [RMFS_Architecture_ko.tex](Architecture/RMFS_Architecture_ko.tex). Mathematical arguments and empirical findings are distinguished below. Documented proofs apply under their stated assumptions and do not guarantee every physical operating situation.

## 1. Model and terminology

**AchievedDate: 2026-11-19**

- Formulated the problem by abstracting time while preserving path-progress order. **AchievedDate: 2026-11-19**
- Defined object occupancy, subpath sweeps, point sweeps, and full-path sweeps. **AchievedDate: 2026-11-19**
- Represented RMFS sweeps of fixed-orientation rectangular objects moving along coordinate axes as unions of rectangles. Accounted for racks remaining fixed in orientation while robots rotate. **AchievedDate: 2026-11-19**
- Distinguished point-sweep duplication for waiting from sweep merging that groups movement while preserving original vertices and bends. **AchievedDate: 2026-11-19**
- Extended paired paths to n-object tuple paths. Each component is a subpath; a point path represents waiting. **AchievedDate: 2026-11-19**

## 2. Execution properties of interference-free tuple paths

**AchievedDate: 2026-11-19**

- Argued that a tuple can execute without inter-object collision if every distinct pair of sweeps is disjoint, including point sweeps of waiting objects. **AchievedDate: 2026-11-19**
- Established finite-time collision-free completion when finitely many tuples execute in order and every movement finishes in finite time. **AchievedDate: 2026-11-19**
- Established a sufficient construction using point-sweep duplication under the strong condition that start and end point sweeps are disjoint from all other objects' full sweeps. This is not claimed to be necessary. **AchievedDate: 2026-11-19**

## 3. Unit path pairs and 16 endpoint types

**AchievedDate: 2026-11-19**

- Classified 16 types using four binary indicators of start/end interference for two objects. **AchievedDate: 2026-11-19**
- Documented a classification and proof of seven feasible and nine infeasible types under additional assumptions concerning one connected common centerline subpath and endpoint interference. **AchievedDate: 2026-11-19**
- Distinguished start-interference, end-interference, and blocking-interference pairs. **AchievedDate: 2026-11-19**
- Used counterexamples to show that centerline intersection, sweep intersection, and interference-progress intervals are not generally equivalent, and restricted the scope accordingly. **AchievedDate: 2026-11-19**

### Empirical verification

On a 4x4 point grid with 2m spacing, 1.5m square objects, and simple paths of at most five edges:

- Checked 1,799,096 unit or independent ordered path pairs. **AchievedDate: 2026-11-19**
- Found no disagreements between the classification formula and a separate reachability check. **AchievedDate: 2026-11-19**
- Cross-checked 320 representative cases with the actual WaitDuplicationSolver. **AchievedDate: 2026-11-19**

This verifies the stated finite experimental domain; it is not an exhaustive proof for arbitrary sweeps.

## 4. Precedence requirements for multiple objects

**AchievedDate: 2026-11-19**

- Confirmed through cyclic-wait cases that pairwise feasibility does not guarantee feasibility for all objects together. **AchievedDate: 2026-11-19**
- Distinguished static serial-passage requirements from actual waiting caused by current occupancy. **AchievedDate: 2026-11-19**
- Established a sufficient condition and proof constructing a complete execution by point-sweep duplication when the static requirement graph is acyclic. **AchievedDate: 2026-11-19**
- Found feasible examples with static cycles. Static cycle detection is therefore not a general infeasibility test. **AchievedDate: 2026-11-19**

## 5. Sweep merging for n objects

**AchievedDate: 2026-11-19**

- Documented componentwise connectivity, ordered concatenation, and union-of-sweeps formulas for two consecutive n-object tuples. **AchievedDate: 2026-11-19**
- Formulated a conservative merge condition requiring disjoint merged sweeps for every distinct object pair. **AchievedDate: 2026-11-19**
- Specified the additional cross terms to check when the original tuples are individually interference-free. **AchievedDate: 2026-11-19**
- Documented the property and proof that safe merging preserves safety and completion under finite-time execution assumptions. **AchievedDate: 2026-11-19**
- Restricted this property to execution of merged validated tuples. It is not a proof of general completion for the entire current asynchronous control algorithm. **AchievedDate: 2026-11-19**

## 6. Recent implementation principles and observations

**AchievedDate: 2026-11-19**

- Agreed to apply merging to movement commands while retaining original control subpath sweeps and departure-decision rules. **AchievedDate: 2026-11-19**
- Confirmed the reasoning that releasing passed portions of fixed paths can shrink or split connected components of the residual interference graph. Formal definitions, theorem statements, and written proofs remain to be completed. **AchievedDate: 2026-11-19**
- Discussed independent treatment of objects whose full paths interfere with no others and of distinct interference components. **AchievedDate: 2026-11-19**
- Identified the scope boundaries: destination point sweeps remain occupied, and assigning new paths can join interference groups again. **AchievedDate: 2026-11-19**

See [ToDoListTheory_en.md](ToDoListTheory_en.md) for remaining theory tasks.
