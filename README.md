# Introduction to ROS

This repository holds the development environment and demos used in the Introduction to ROS video series.

## Getting Started

## Required Software

Before you start, install the following programs on your computer:

 * (Windows) [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install)
 * [Docker Desktop](https://www.docker.com/products/docker-desktop/)

### Build Docker Image

From this directory, build the image (this will take some time):

```sh
docker build -t env-ros2 .
```

Run the image:

Linux/macOS:

```sh
docker run --rm -it -e PUID=$(id -u) -e PGID=$(id -g) -p 22002:22 -p 3000:3000 -v "${PWD}/workspaces:/config/workspaces" env-ros2
```

Windows (PowerShell):

```bat
docker run --rm -it -e PUID=$(wsl id -u) -e PGID=$(wsl id -g) -p 22002:22 -p 3000:3000 -v "${PWD}\workspaces:/config/workspaces" env-ros2
```

Alternatively, you can run the image in interactive mode by adding the `--entrypoint /bin/bash` argument. This will allow you to skip running the VS Code server in the background.

### Connect to Container

With the Docker image built, you have a few options to connect to the development environment: browser or SSH. Choose one of the options below. Note that the *Dev Containers* extension for VS Code does not seem to be working at this time. If you manage to get it working, please let me know in an issue or pull request!

#### Option 1: Connect via Browser

Open a browser and navigate to http://localhost:8800/.

#### Option 2: VS Code SSH

If you want to develop Zephyr applications using your local instance of VS Code, you can connect to the running container using SSH. This will allow you to use your custom themes, extensions, settings, etc.

In your local VS Code, install the [Remote - SSH extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh).

Open the extension in VS Code and create a new connection: **root@localhost:22002**.

Connect and login using the password in the Dockerfile (default: `fota`). Go to **File > Open Workspace from File..** and select the **/esp-idf.code-workspace** file when prompted. Enter the password again if requested. This should configure your VS Code workspace with the */workspace* directory mapped from the host directory alongside the required toolchain directories (e.g. */opt/toolchains/esp-idf*).

## License

All software in this repository, unless otherwise noted, is licensed under the [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) license.
