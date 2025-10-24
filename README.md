# Crane Panel Builder Workspace

This workspace contains two ROS 2 Humble packages that work together to sequence
wall and column panel pick-and-place motions for a crane using only the IFC
GUID, the panel's pick position, the placement target position, and the panel
chain's `NEXT` linkage.

## Packages

- `crane_interfaces`: defines the `PanelTask` message that carries only the
  required information (`ifc_guid`, `panel_position`, `target_position`, and
  `next_ifc_guid`).
- `crane_builder`: provides two Python nodes for sequencing panel moves:
  - `panel_chain_executor` reads a YAML description of your wall or column
    panels and publishes `PanelTask` messages in strict NEXT-chain order.
  - `neo4j_panel_chain_executor` queries a Neo4j database for panels that carry
    the `ifcGuid`, `HookPoint`, `PanelPosition`, `TargetPosition`, and
    `SequenceIndex` properties, then publishes the tasks with the exact
    NEXT-chain defined in Neo4j.
  Both nodes publish on the `panel_task` topic and assume an identity
  orientation for every pick and place (no rotation is applied).

## Usage

### Build the workspace (all packages)

1. Build the workspace and source the overlay:
   ```bash
   cd /path/to/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

### Run the RCAN demo orchestrator

The RCAN orchestrator (`rcan_core`) wires together the in-process message broker,
Neo4j-backed panel queries, template-based ROS node generation, and a simulated
robot executor. The snippet below shows a full shell session that builds the
workspace, configures Neo4j credentials, and launches the demo end to end:

```bash
cd /path/to/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Point the orchestrator at your Neo4j instance
export RCAN_NEO4J_URI=bolt://localhost:7687
export RCAN_NEO4J_USER=neo4j
export RCAN_NEO4J_PASSWORD=secret

# (Optional) choose the exact panel GUID to install
export RCAN_DEMO_IFCGUID=X1

# Run the end-to-end workflow
python -m rcan_core.main_rcan
```

After the command finishes you will find the generated node under
`src/rcan_nodes/rcan_nodes/generated/` (for example,
`install_panel_X1.py`) and the panel state recorded back in Neo4j via the state
manager service.

For additional context, the ordered list below explains the prerequisites in
more detail:

1. Ensure your Neo4j instance contains panels with the `Panel` label and the
   `ifcguid`, `HookPoint`, and `TargetPosition` properties required by the
   database API (see `src/rcan_core/rcan_core/db_api.py` for details). If you do
   not have a live database, the test fixture in
   `src/rcan_core/tests/test_main_rcan.py` shows how to mock the driver.
2. Export the Neo4j connection settings expected by `rcan_core.config` (shown
   in the snippet above).
3. (Optional) set `RCAN_DEMO_IFCGUID` to force a specific panel.
4. Launch the orchestrator entry point (`python -m rcan_core.main_rcan`).

On success the demo:

- Requests a panel installation via the smart component API.
- Generates a ROS node script in `src/rcan_nodes/rcan_nodes/generated/`.
- Simulates the robot executing the task (via the generated node) and reports
  the resulting panel state on the in-process broker.

You can rerun the command with a different `RCAN_DEMO_IFCGUID` value to generate
additional nodes; each run overwrites the panel's status in Neo4j through the
state manager service.

### YAML-based panel chain executors

The original `crane_builder` package still provides two panel-chain executors.
To use the YAML-driven node:

1. Edit `src/crane_builder/config/example_panels.yaml` (or create your own file)
   so that it contains one entry per panel with the following keys only:
   - `ifc_guid`: unique identifier of the panel.
   - `panel_position`: `[x, y, z]` pick location of the panel.
   - `target_position`: `[x, y, z]` placement location of the panel.
   - `next`: the IFC GUID of the next panel in the chain, or empty/`null` for the
     final element.
2. Launch the YAML-based executor:
   ```bash
   ros2 run crane_builder panel_chain_executor \
     --ros-args -p panel_chain_file:=/absolute/path/to/your_panels.yaml
   ```

The node publishes each `PanelTask` sequentially, allowing your crane control
stack to consume the topic and execute the pick-and-place motions without any
additional parameters or orientation handling.

### Neo4j-driven execution (legacy)

To source the panel information directly from Neo4j using the legacy node,
launch the Neo4j executor. The node expects every panel node (label defaults to
`Panel`) to provide:

- `ifcGuid`: unique identifier of the panel.
- `HookPoint`: pick location (the hook/grab point) as either `[x, y, z]` or a
  mapping with `x`, `y`, `z` keys.
- `PanelPosition`: the panel's center location, used for logging.
- `TargetPosition`: `[x, y, z]` placement location of the panel.
- `SequenceIndex`: installation order number.
- A `NEXT` relationship (configurable) linking each panel to the next one in the
  chain.
