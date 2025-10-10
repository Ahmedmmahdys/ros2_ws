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

1. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
2. Edit `src/crane_builder/config/example_panels.yaml` (or create your own file)
   so that it contains one entry per panel with the following keys only:
   - `ifc_guid`: unique identifier of the panel.
   - `panel_position`: `[x, y, z]` pick location of the panel.
   - `target_position`: `[x, y, z]` placement location of the panel.
   - `next`: the IFC GUID of the next panel in the chain, or empty/`null` for the
     final element.
3. Launch the YAML-based executor:
   ```bash
   ros2 run crane_builder panel_chain_executor \
     --ros-args -p panel_chain_file:=/absolute/path/to/your_panels.yaml
   ```

The node publishes each `PanelTask` sequentially, allowing your crane control
stack to consume the topic and execute the pick-and-place motions without any
additional parameters or orientation handling.

### Neo4j-driven execution

To source the panel information directly from Neo4j instead of a YAML file,
launch the Neo4j executor. The node expects every panel node (label defaults to
`Panel`) to provide:

- `ifcGuid`: unique identifier of the panel.
- `HookPoint`: pick location (the hook/grab point) as either `[x, y, z]` or a
  mapping with `x`, `y`, `z` keys.
- `PanelPosition`: the panel's center location, used for logging.
- `TargetPosition`: `[x, y, z]` placement location of the panel.
- `SequenceIndex`: installation order number.
- A `NEXT` relationship (configurable) linking each panel to the next one in
  the chain.

Example invocation:

```bash
ros2 run crane_builder neo4j_panel_chain_executor \
  --ros-args \
    -p uri:=bolt://neo4j.example.com:7687 \
    -p user:=neo4j \
    -p password:=secret \
    -p panel_label:=Panel \
    -p next_relationship:=NEXT
```

The executor orders the panels using `SequenceIndex`, verifies the declared
`NEXT` chain matches that order, and publishes each `PanelTask` sequentially.
The published `panel_position` corresponds to Neo4j's `HookPoint`, ensuring the
pick uses the precise grab location provided by the database.
