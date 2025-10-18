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
    ├── crane_builder/           # Python package with the Neo4j executor.
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

> **Note:** This workspace targets ROS 2 Humble exclusively and is built with
> `colcon`/ament. Catkin (the ROS 1 build system) is not required or used
> anywhere. If you see lingering `CATKIN_*` cache variables from an older ROS 1
> environment, remove them or start a fresh shell sourced only with `/opt/ros/
> humble/setup.bash` before running `colcon build`.

Implement any extra Python nodes inside `src/crane_builder/crane_builder/` and
add supporting resources as needed within the package. New interface definitions
should live in `src/crane_interfaces` beside `PanelTask.msg`.

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
  required information (`ifc_guid`, `hook_point`, `panel_position`,
  `target_position`, and `next_ifc_guid`).
- `crane_builder`: provides the Neo4j-backed Python node that sequences panel
  moves. The `neo4j_panel_chain_executor` queries a Neo4j database for panels
  hosted on a specific level's walls or columns, using each panel's
  `ifcGuid`, `HookPoint`, `PanelPosition`, and `TargetPosition` fields to
  publish the NEXT-chain defined in Neo4j. Every message goes to the
  `panel_task` topic and assumes an identity orientation (no rotation is
  applied).

## Usage

### 1. Common ROS 2 Humble setup

All packages target ROS 2 Humble APIs. Start every new shell session by sourcing
your Humble installation:

```bash
source /opt/ros/humble/setup.bash
```

Install the single Python dependency once (normally handled by `rosdep`):

```bash
pip install --user neo4j
```

Build the workspace from the repository root. If you previously built another
copy of these packages in the same workspace, clear the old artifacts first:

```bash
rm -rf build install log  # optional: only if you're cleaning an old build
colcon build --symlink-install
source install/setup.bash
```

Run the commands from the root that owns the `src/` directory shown earlier. If
`colcon` reports duplicate packages, remove or rename any other workspaces on
disk that still contain these package folders so only one copy remains.

### 2. Stream panel tasks from Neo4j

Panel tasks always originate from Neo4j. The executor reads the active NEXT
chain directly from the graph so you never need to maintain a parallel YAML
representation. Ensure each `FormworkPanel` node exposes the required fields and
links to the next element through a dedicated `NEXT_*` relation (for example
`NEXT_1`, `NEXT_2`, …). After the common setup, launch the executor with:

```bash
ros2 run crane_builder neo4j_panel_chain_executor
```

Override connection settings or select a different chain at the command line as
needed:

```bash
ros2 run crane_builder neo4j_panel_chain_executor \
  --ros-args \
    -p uri:=neo4j+s://ae1083a1.databases.neo4j.io \
    -p user:=neo4j \
    -p password:=<your-password> \
    -p database:=neo4j \
    -p chain_relation:=NEXT_5
```

The Neo4j executor is now the sole source of panel tasks, keeping the workspace
aligned with the production graph data.

### 3. Run the RCAN executor

After sourcing the workspace overlay you can start the RCAN executor to observe
incoming panel tasks streamed from Neo4j:

```bash
ros2 run rcan_executor rcan_executor
```

The node subscribes to the `panel_task` topic by default. Use the
`panel_task_topic` parameter to point it at a different topic if needed:

```bash
ros2 run rcan_executor rcan_executor --ros-args -p panel_task_topic:=/my_topic
```

- `ifcGuid`: unique identifier of the panel.
- `HookPoint`: pick location (the hook/grab point) as either `[x, y, z]` or a
  mapping with `x`, `y`, `z` keys.
- `PanelPosition`: the panel's center location, used for logging.
- `TargetPosition`: `[x, y, z]` placement location of the panel.
- A `NEXT_*` relationship (for example `NEXT_1`, `NEXT_2`, …) linking each panel
  to the next one in its wall/column chain. Each chain now lives in its own
  relation type, so you can query any wall or column directly by selecting the
  corresponding `NEXT_*` relation name.

The executor defaults to the hosted Aura instance requested above
(`neo4j+s://ae1083a1.databases.neo4j.io` with the provided managed credentials)
and publishes whichever chain you request via the `chain_relation` parameter.
If you omit the parameter the node sequences the panels connected by
`NEXT_1`.

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
    -p chain_relation:=NEXT_12
```

The executor gathers the panels connected by the requested `NEXT_*` relation,
following each chain from its head node to the tail node. Every link is
published in order: the `hook_point` of the outgoing `PanelTask` message
matches Neo4j's `HookPoint`, while `panel_position` mirrors `PanelPosition` so
your crane has access to both the grab location and the panel centre.
