# Crane Panel Builder Workspace

This repository is already structured as a ROS 2 Humble workspace. Clone (or
copy) it into the location where you normally keep workspaces—e.g.
`~/ros2_ws`—then run the usual `colcon` commands from the repository root (the
directory that contains this `README.md` and the `src/` folder). The workspace
contains two ROS 2 Humble packages that work together to sequence
wall and column panel pick-and-place motions for a crane using only the IFC
GUID, the panel's pick position, the placement target position, and the panel
chain's `NEXT` linkage.

## Workspace layout

```
ros/
├── README.md                     # You are here.
└── src/
    ├── crane_builder/           # Python package with the panel executors.
    │   ├── config/              # Example YAML panel chain.
    │   └── crane_builder/       # Python modules (nodes & utilities).
    └── crane_interfaces/        # Message package with PanelTask.msg.
```

Both packages live directly under the workspace `src/` directory. Python-only
packages such as `crane_builder` keep their code inside a folder that exposes a
`setup.py`/`setup.cfg` (an **`ament_python`** package). Interface or CMake-based
packages such as `crane_interfaces` live beside them and provide a
`CMakeLists.txt` (an **`ament_cmake`** package). If you create additional
packages, place each package folder at this same level—`src/<package_name>/`—and
pick the build type (`ament_python` or `ament_cmake`) based on whether the
package is pure Python or requires CMake/ROS IDL generation.

Place any additional configuration files next to the provided examples under
`src/crane_builder/config/`, and implement any extra Python nodes inside
`src/crane_builder/crane_builder/`. New interface definitions should live in
`src/crane_interfaces` beside `PanelTask.msg`.

### Using the packages inside another workspace

If you already maintain a workspace such as `~/ros2_ws` and want these packages
to live under `~/ros2_ws/src/`, you have two equivalent options:

1. **Treat this repository as the workspace** (recommended). Clone it to
   `~/ros2_ws` so that the repository root *is* your workspace root:
   ```bash
   cd ~
   git clone <this-repo-url> ros2_ws
   cd ros2_ws
   colcon build --symlink-install
   ```
   This keeps the provided `src/` directory intact and avoids nesting workspaces.

2. **Copy the packages into an existing workspace.** If you already have
   `~/ros2_ws/src/`, you can move the two packages into that directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <this-repo-url> crane_panel_builder_tmp
   mv crane_panel_builder_tmp/src/crane_builder .
   mv crane_panel_builder_tmp/src/crane_interfaces .
   rm -rf crane_panel_builder_tmp
   cd ..
   colcon build --symlink-install
   ```
   In this layout you keep using your original workspace root while reusing the
   packages from this repository.

## Packages

- `crane_interfaces`: defines the `PanelTask` message that carries only the
  required information (`ifc_guid`, `panel_position`, `target_position`, and
  `next_ifc_guid`).
- `crane_builder`: provides two Python nodes for sequencing panel moves:
  - `panel_chain_executor` reads a YAML description of your wall or column
    panels and publishes `PanelTask` messages in strict NEXT-chain order.
  - `neo4j_panel_chain_executor` queries a Neo4j database for panels hosted on
    a specific level's walls or columns, using each panel's `ifcGuid`,
    `HookPoint`, `PanelPosition`, and `TargetPosition` fields to publish the
    NEXT-chain defined in Neo4j.
  Both nodes publish on the `panel_task` topic and assume an identity
  orientation for every pick and place (no rotation is applied).

## Usage

1. Source your ROS 2 Humble environment (skip if your shell already sources it):
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Install the Python dependencies (only required once, normally handled by
   `rosdep`; included here for clarity when running inside a fresh workspace):
   ```bash
   pip install --user neo4j
   ```
3. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
4. Edit `src/crane_builder/config/example_panels.yaml` (or create your own file)
   so that it contains one entry per panel with the following keys only:
   - `ifc_guid`: unique identifier of the panel.
   - `panel_position`: `[x, y, z]` pick location of the panel.
   - `target_position`: `[x, y, z]` placement location of the panel.
   - `next`: the IFC GUID of the next panel in the chain, or empty/`null` for the
     final element.
5. Launch the YAML-based executor:
   ```bash
   ros2 run crane_builder panel_chain_executor \
     --ros-args -p panel_chain_file:=/absolute/path/to/your_panels.yaml
   ```

The node publishes each `PanelTask` sequentially, allowing your crane control
stack to consume the topic and execute the pick-and-place motions without any
additional parameters or orientation handling.

### Neo4j-driven execution

To source the panel information directly from Neo4j instead of a YAML file,
launch the Neo4j executor (after completing steps 1–3 above). The node expects
every panel node (labelled `FormworkPanel` in the Aura dataset) to provide:

- `ifcGuid`: unique identifier of the panel.
- `HookPoint`: pick location (the hook/grab point) as either `[x, y, z]` or a
  mapping with `x`, `y`, `z` keys.
- `PanelPosition`: the panel's center location, used for logging.
- `TargetPosition`: `[x, y, z]` placement location of the panel.
- A `NEXT` relationship linking each panel to the next one in the chain.

Panels must also be connected to the level you plan to install via the
relationships created by the IFC importer:

- Walls: `(:Level)-[:HAS_WALL]->(:Wall)-[:HAS_PART]->(:WallPart)` and each
  panel is `(:FormworkPanel)-[:HOSTED_BY]->(:WallPart)`.
- Columns: `(:Level)-[:HAS_COLUMN]->(:Column)` and each panel is
  `(:FormworkPanel)-[:HOSTED_BY]->(:Column)`.

Provide the level name (matches the `Level.name` property) and the mode you want
to sequence (`Wall` or `Column`) as ROS parameters. The executor defaults to the
hosted Aura instance requested above (`neo4j+s://ae1083a1.databases.neo4j.io`
with the provided managed credentials), so you can launch it without overriding
the connection settings:

```bash
ros2 run crane_builder neo4j_panel_chain_executor
```

Example invocation with explicit overrides:

```bash
ros2 run crane_builder neo4j_panel_chain_executor \
  --ros-args \
    -p uri:=bolt://neo4j.example.com:7687 \
    -p user:=neo4j \
    -p password:=secret \
    -p level_name:="Level 01" \
    -p mode:=Wall
```

The executor gathers one chain per wall/column head on the specified level by
following the `NEXT` relationship all the way to each chain's tail. Each chain
is published in order, and the `panel_position` of the outgoing `PanelTask`
messages corresponds to Neo4j's `HookPoint`, ensuring the pick uses the precise
grab location provided by the database.
