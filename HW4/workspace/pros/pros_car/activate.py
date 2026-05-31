#!/usr/bin/env python3

import os
import sys
import platform
import subprocess
import argparse
import json


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Run PROS car Docker container")
    parser.add_argument(
        "--port",
        nargs=2,
        metavar=("HOST_PORT", "CONTAINER_PORT"),
        help="Port mapping in format HOST_PORT:CONTAINER_PORT",
    )
    parser.add_argument(
        "--x11",
        action="store_true",
        help="Enable X11 forwarding for GUI applications",
    )
    args = parser.parse_args()

    # Set up volume arguments
    volume_args = f"-v {os.getcwd()}/src:/workspaces/src -v {os.getcwd()}/launch:/workspaces/launch"

    # Handle port mapping
    port_mapping = ""
    if args.port:
        host_port, cont_port = args.port
        port_mapping = f"-p {host_port}:{cont_port}/udp"

    # Handle X11 display forwarding
    x11_args = ""
    if args.x11 and platform.system() != "Darwin":  # X11 forwarding works on Linux
        x11_args = f"-e DISPLAY={os.environ.get('DISPLAY', ':0')} -v /tmp/.X11-unix:/tmp/.X11-unix"

    # Detect system architecture and OS
    arch = platform.machine()
    os_type = platform.system()
    print(f"Detected OS: {os_type}, Architecture: {arch}")

    # Initialize GPU related variables
    gpu_flags = ""
    use_gpu = False

    # Check for GPU support on Linux
    if os_type == "Linux":
        if os.path.isfile("/etc/nv_tegra_release"):
            gpu_flags = "--runtime=nvidia"
            use_gpu = True
        else:
            # Check for NVIDIA runtime in Docker
            try:
                docker_info = subprocess.check_output(
                    ["docker", "info", "--format", "{{json .}}"]
                )
                if b'"Runtimes"' in docker_info and b"nvidia" in docker_info:
                    gpu_flags = "--gpus all"
                    use_gpu = True
            except subprocess.CalledProcessError:
                use_gpu = False

    # Test if GPU is available
    if use_gpu:
        print("Testing Docker run with GPU...")
        try:
            subprocess.check_call(
                f'docker run --rm {gpu_flags} ghcr.io/screamlab/pros_car_docker_image:latest /bin/bash -c "echo GPU test"',
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError:
            print("GPU not supported or failed, disabling GPU flags.")
            gpu_flags = ""
            use_gpu = False

    print(f"GPU Flags: {gpu_flags}")

    # Set up device options
    device_options = ""
    for device in ["/dev/usb_front_wheel", "/dev/usb_rear_wheel", "/dev/usb_robot_arm"]:
        if os.path.exists(device):
            device_options += f" --device={device}"

    # Run Docker based on architecture
    if arch == "aarch64":
        print("Detected architecture: arm64")
        docker_cmd = (
            f"docker run -it --rm "
            f"--network compose_my_bridge_network "
            f"{port_mapping} "
            f"{device_options} "
            f"{x11_args} "
            f"--runtime=nvidia "
            f"--env-file .env "
            f'-v "{os.getcwd()}/src:/workspaces/src" '
            f"ghcr.io/screamlab/pros_car_docker_image:latest "
            f"/bin/bash"
        )
        os.system(docker_cmd)

    elif arch == "x86_64" or (arch == "arm64" and os_type == "Darwin"):
        print("Detected architecture: amd64 or macOS arm64")

        if os_type == "Darwin":
            print("Running Docker on macOS (without GPU support)...")
            docker_cmd = (
                f"docker run -it --rm "
                f"--network compose_my_bridge_network "
                f"{port_mapping} "
                f"{device_options} "
                f"--env-file .env "
                f"{volume_args} "
                f"ghcr.io/screamlab/pros_car_docker_image:latest "
                f"/bin/bash"
            )
            os.system(docker_cmd)
        else:
            print("Trying to run with GPU support...")
            docker_cmd = (
                f"docker run -it --rm "
                f"--network compose_my_bridge_network "
                f"{port_mapping} "
                f"{x11_args} "
                f"{gpu_flags} "
                f"{device_options} "
                f"--env-file .env "
                f"{volume_args} "
                f"ghcr.io/screamlab/pros_car_docker_image:latest "
                f"/bin/bash"
            )

            # Try with GPU first
            if os.system(docker_cmd) != 0:
                print("GPU not supported or failed, falling back to CPU mode...")
                # Fall back to CPU mode
                docker_cmd = (
                    f"docker run -it --rm "
                    f"--network compose_my_bridge_network "
                    f"{port_mapping} "
                    f"{x11_args} "
                    f"--env-file .env "
                    f"{device_options} "
                    f"{volume_args} "
                    f"ghcr.io/screamlab/pros_car_docker_image:latest "
                    f"/bin/bash"
                )
                os.system(docker_cmd)
    else:
        print(f"Unsupported architecture: {arch}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
