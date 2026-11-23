# RMFS viewer convention

- Use a white background by default in every RMFS animation and screenshot.
- Keep labels, status text, and grid lines readable on white.
- The user publishes screenshots in papers; dark backgrounds consume too much printing ink.
- Do not require a command-line flag to obtain the white background. Preserve this default in new examples and viewers.
- Shared viewer controls apply to every example: H toggles the information overlay while keeping current simulation time visible; +/- zoom; arrow keys pan; T/I select Top/Iso. Preserve zoom and pan across view switches.
- P pauses/resumes playback; V switches rack/chassis wire and surface modes. Default to wire mode; eyes and pupils remain polygon surfaces in both modes.
- Each robot has a stable color shared by its chassis, route and carried rack. Waiting does not change that color; collision highlighting may temporarily override the chassis color.

# Fixed-path traffic policy (November 20, 2026)

- The selected operating policy for flexible traffic decisions is FIFO.
- When a single object reaches an available crossing, admit it as soon as the existing safety and required-order checks permit. Do not intentionally hold it for a later arrival.
- When a resource is released and multiple eligible objects are waiting, select the earliest waiter. FIFO does not override mandatory tuple precedence, physical occupancy, or safety checks.
- Do not apply snapshot lookahead, LP, or Random as the default policy at this fixed-path traffic-selection level.
- Preserve the historical policy-comparison code and results as experiments; their availability does not make them operating defaults.
- Snapshot lookahead for choosing among different routes is a separate, upcoming RobotPathPlanner study. Discuss its design with the user before implementing new architecture.

# Actual-clearance admission (November 21, 2026)

- Do not use local collision-prediction lookahead to authorize passage before a conflicting sweep is actually released. A predicted timing gap is not an operating admission condition.
- Keep FIFO, safe-prefix approach, per-point PSensor releases, and residual-sweep updates. Dispatch eligible waiters on actual release reports.
- Historical predictive passage results may remain, but obsolete request/report lookahead code has been removed. The 03_08 example uses the ordinary DEVS engine without a motion-status port.
