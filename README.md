# Hybrid Control Car

The goal of this project is to make a car that is controlled using a neural network for pathfinding in an outer control loop and a model predicitve control inner loop for steering the car from one path point to the next. It is currently a heavy work-in-progress.

See [project.pdf](./project.pdf) for more details.

## Running the Project

Since the underlying physics simulation is written in the Rust programming language, you will need to [install Rust](https://doc.rust-lang.org/book/ch01-01-installation.html#installation) in addition to Anaconda. You will also need to install Maturin with `pip install maturin`.

If you're running this in VSCode, you will need to install the `rust-analyzer` extension and the official (by Microsoft) extensions for Jupyter notebook. Otherwise, install the appropriate extensions for your editor of chose.

To build the physics simulation package, simply run `maturin develop` in the virtual environment you've created to run the package. Note that if you've ran the notebook and want to rebuild, you will need to restart the kernel.