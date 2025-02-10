# Use an official Ubuntu base image with common dev tools
FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for C++ and Python
RUN apt-get update && apt-get install -y \
    cmake \
    ninja-build \
    g++ \
    python3 \
    python3-pip \
    python3-venv \
    clang \
    lldb \
    gdb \
    libgtest-dev \
    pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --upgrade pip && pip install \
    pytest \
    build \
    twine \
    pybind11

# Set the default working directory inside the container
WORKDIR /workspace

# Create a virtual environment for Python (optional)
RUN python3 -m venv /workspace/venv
