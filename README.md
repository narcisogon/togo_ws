# Togo Containerized Workspace

Basic dockerized workspace for the EG Husky Togo.
The contents of the `src` directory should be treated similarly to a "normal" ROS workspace.
That is, source code can be imported and added as needed to `src/`, then be built and run inside of an isolated, ROS enabled environment.

This workflow has been tested against the `jazzy` ROS distro.
To change ROS versions, update the `ROS2_DISTRO` variable in your environment.
Note the `2`! As this is intended to be isolated from your system.

## Quick Development Setup

1) Install Docker
    - Don't worry about Docker Desktop
    - For Ubuntu recommend using the [utility script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
    - After running the utility script, you should run the [post-installation steps for linux](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user), which helps manage the user settings and running without root access.
2) ***VERY IMPORTANT*** Recursively initialize all submodules.  Note that the fixposition and seyond driver packages contain many nested submodules, so the `--recursive` flag is ***critical***.

    ```bash
    git submodule update --init --recursive
    ```

3) Setup additional source code for the `src/` directory (if you need them)
    - ie, `git submodule add ...`

4) Set your user information for the project build
    - We recommend just putting this in your `~/.bashrc`:

      ```bash
      export USER_UID=$(id -u $USER)
      export USER_GID=$(id -g $USER)
      ```

    - Alternatively, open the `.env` file in the root of this repo and update each line with your information
        - `USER_UID` and `USER_GID`
            - found using `id -u` and `id -g` respectively

## Using the Images

***VERY IMPORTANT*** Apply the required pre-build steps on the host by running the following script from the repo root:

```bash
./scripts/pre_build.sh
```

We provide two Togo images, one for [local development](#development-image) and one for [development on hardware](#hardware-development-image).
The images can be used the same way;
the hardware container just includes different volume mounts and brings up the micro-ROS agent for Togo.

Once you're attached to the container, you can use it as a regular colcon workspace (see [building the workspace](#building-the-togo-workspace)).
The contents of the `src/` directory will be mounted into `/home/er4-user/ws/src`.

### Development Image

Build the development image from the repo root, and then launch it:

```bash
# Compile the image
docker compose build dev

# Start it
docker compose up dev -d

# Connect to the console
docker compose exec dev terminator
```

### Hardware Development Image

Build the hardware development image from the repo root, and then launch it:

```bash
# Compile the image
docker compose build hw-dev

# Start it; this container will automatically start the micro-ROS agent docker container as well
docker compose up hw-dev -d

# Connect to the console
docker compose exec hw-dev terminator
```

### Building the Togo Workspace

Once you're attached to the container, build the workspace as normal:

```bash
colcon build
```

This workspace depends drivers for several sensors, namely fixposition and seyond.
These packages will complain when building, and will include messages marked "fatal".
Ignore this; the build should complete just fine, the packages are just whiny.

For awareness, both the [fixposition](https://docs.fixposition.com/fd/installation-and-usage#Installationandusage-a%29SetupdriverforanexistingROSworkspace) and [seyond](https://github.com/Seyond-Inc/seyond_ros_driver/blob/main/src/seyond_lidar_ros/README.md#compile) require extra build steps.
These are handled by the `pre_build.sh` script run before building the docker images.
By the time you attach to the container, these packages can be built as expected within a ROS workspace.

For more information on running applications refer to Togo's [README.md](./src/togo/README.md).
To get started, we recommend [Gazebo instructions](./src/togo/README.md#gazebo) for the dev image and [hardware instructions](./src/togo/README.md#deploy) for the hardware image.

## Other Things to Note

- Build logs, compiled artifaces, and the `.ccache` are also mounted in the workspace/user home.
This ensure artifacts are persisted even when restarting or recreating the container.

- The `.bash` folder gets mounted into your workspace, and the environment variable `HISTFILE` is set in the docker compose file.
This points the bash to keep the history in this folder, which will persist between docker container sessions so that your history is kept.

- Your host's DDS configuration (either cyclone or fastrtps) will be mounted into the image if set in your environment.
For more information refer to the [compose specification](docker-compose.yaml).

- Defaults for `colcon build` are set for the user. To change or modify, refer to the [defaults file](config/colcon-defaults.yaml).

- We use [MuJoCo](https://mujoco.readthedocs.io/en/stable/XMLreference.html) for many of our dynamic simulations, so we include installing in the [Dockerfile](./Dockerfile).

## Troubleshooting

Common pitfalls and troubleshooting tips are documented in the [troubleshooting guide](./docs/TROUBLESHOOTING.md).
