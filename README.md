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

## Step-by-step run guide

The following checklist assumes a clean ROS 2 Humble workstation. Each numbered
step builds on the previous one—walk through them in order whenever you set up a
fresh machine or want to verify an existing installation.

### 1. Prepare a clean shell

Open a new terminal and source only the Humble environment (avoid mixing in ROS 1
or other overlays):

```bash
source /opt/ros/humble/setup.bash
```

If you had previously sourced another workspace in this terminal, open a new
tab/window instead so no stale `CATKIN_*` cache entries leak into the Humble
build.

### 2. Position the workspace

Change into the workspace root—the directory that contains this `README.md` and
the `src/` folder. If you cloned the repository to `~/ros2_ws`, that command is:

```bash
cd ~/ros2_ws
```

Run `ls` and confirm you see the `src/` directory with the three packages inside
(`crane_builder`, `crane_interfaces`, `rcan_executor`). If you have other copies
of the same packages elsewhere on disk, remove or rename them to avoid colcon's
duplicate-package guard.

### 3. Install Python dependencies (one time)

Install the Neo4j Python driver in your user site-packages. Skip this step if
`pip` reports that the requirement is already satisfied:

```bash
pip install --user neo4j
```

### 4. Optionally clean old build artifacts

If you are reusing a workspace that was built before, remove the `build/`,
`install/`, and `log/` directories so the next build starts from a blank slate:

```bash
rm -rf build install log
```

### 5. Build the workspace with colcon

From the workspace root run:

```bash
colcon build --symlink-install
```

The build should finish without warnings about `CATKIN_*` variables. If you do
see such warnings, double-check that the terminal was sourced only with Humble
(`step 1`) and that no other workspaces remain in your current `AMENT_PREFIX_PATH`.

### 6. Source the workspace overlay

After a successful build, overlay the generated setup file so your shell knows
about the newly built packages:

```bash
source install/setup.bash
```

Whenever you open a new terminal, repeat steps 1 and 6 so both the base Humble
environment and this workspace are sourced.

### 7. Configure Neo4j access

Ensure the Neo4j instance that holds your panel data is reachable. The executor
expects each `FormworkPanel` node to provide `ifcGuid`, `HookPoint`,
`PanelPosition`, and `TargetPosition` properties and to connect to the next panel
via a dedicated `NEXT_*` relationship (for example `NEXT_1`). Keep the URI,
username, password, database name, and relation type handy—you will pass them as
parameters in the next step if they differ from the defaults.

### 8. Stream panel tasks from Neo4j

Start the Neo4j-backed executor. It publishes `crane_interfaces/PanelTask`
messages to the `panel_task` topic in the same order they appear in the selected
chain:

```bash
ros2 run crane_builder neo4j_panel_chain_executor
```

Override connection details or select another NEXT chain at launch time:

```bash
ros2 run crane_builder neo4j_panel_chain_executor \
  --ros-args \
    -p uri:=neo4j+s://<host>:<port> \
    -p user:=<username> \
    -p password:=<password> \
    -p database:=<database> \
    -p chain_relation:=NEXT_5
```

Leave this node running while downstream consumers subscribe to the
`panel_task` topic.

### 9. Run the RCAN executor

In a second terminal (repeat steps 1 and 6 there), launch the RCAN executor that
listens to the panel tasks:

```bash
ros2 run rcan_executor rcan_executor
```

The node subscribes to `/panel_task` by default. If your deployment uses a
different topic name, override it with:

```bash
ros2 run rcan_executor rcan_executor --ros-args -p panel_task_topic:=/my_topic
```

### 10. Verify the message flow (optional)

Use the ROS 2 CLI to inspect the topic and confirm that messages are flowing:

```bash
ros2 topic list | grep panel_task
ros2 topic echo /panel_task
```

You should see each `PanelTask` emitted by `neo4j_panel_chain_executor`. Stop the
nodes with `Ctrl+C` when you are done.
