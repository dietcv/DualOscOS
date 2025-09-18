# DualOscOS

DualOscOS is a bandlimited dual wavetable oscillator with cross-modulation and optional oversampling. 
It features cross-modulation between the two coupled wavetable oscillators.
The wavetable oscillators are bandlimited via dynamic mipmapping and sinc interpolation.

### Requirements

- CMake >= 3.10
- SuperCollider source code

### Building

Clone the project:

    git clone https://github.com/dietcv/DualOscOS
    cd DualOscOS
    mkdir build
    cd build

Then, use CMake to configure and build it:

    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build . --config Release
    cmake --build . --config Release --target install

You may want to manually specify the install location in the first step to point it at your
SuperCollider extensions directory: add the option `-DCMAKE_INSTALL_PREFIX=/path/to/extensions`.

It's expected that the SuperCollider repo is cloned at `../supercollider` relative to this repo. If
it's not: add the option `-DSC_PATH=/path/to/sc/source`.

### Developing

Use the command in `regenerate` to update CMakeLists.txt when you add or remove files from the
project. You don't need to run it if you only change the contents of existing files. You may need to
edit the command if you add, remove, or rename plugins, to match the new plugin paths. Run the
script with `--help` to see all available options.
