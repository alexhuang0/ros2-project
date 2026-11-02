# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

## Fast robot development loop

Start the development container and simulator from WSL:

```bash
MODE_OF_OPERATION=develop ./watod up -d --no-build gazeboserver robot_dev
MODE_OF_OPERATION=develop ./watod -t robot_dev
```

Inside the container, source the repository shortcuts once per shell:

```bash
source /home/bolty/ament_ws/src/scripts/dev.sh
```

Then use:

```bash
cb    # Build the robot packages and source the install overlay
run   # Build, then launch the robot
```

The script is stored in `scripts/dev.sh`, so it persists across container
restarts and can be edited from VS Code on the WSL side.
