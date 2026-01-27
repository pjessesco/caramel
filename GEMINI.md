# Caramel

Caramel is a physically-based offline renderer written in C++20. It is designed for educational purposes and study, implementing core rendering concepts like path tracing, multiple importance sampling, and acceleration structures.

## Project Overview

*   **Language:** C++20
*   **Build System:** CMake
*   **Base Library:** [Peanut](https://github.com/pjessesco/peanut) (located in `ext/peanut`)
*   **Dependencies:** `nlohmann/json`, `stb_image`, `tiny_obj_loader`, `tinyexr` (all included in `ext/`).

## Directory Structure

*   `src/`: Core implementation source files.
    *   `integrators/`: Rendering algorithms (Path Tracing, Depth, Normal, etc.).
    *   `bsdfs/`: Material models (Diffuse, Mirror, Microfacet, etc.).
    *   `lights/`: Light sources (Area, Point, Environment).
    *   `mesh_accel/`: Acceleration structures (BVH, Octree).
    *   `shapes/`: Geometric primitives (Triangle, OBJ mesh).
    *   `samplers/`: Random number generation strategies.
*   `include/`: Header files corresponding to the source files.
*   `ext/`: External dependencies.
*   `caramel-scenes/`: Test scenes and assets used for validation and examples.
*   `test/`: Unit and integration tests using Catch2.

## Build and Usage

### Prerequisites
*   C++20 compatible compiler (Clang, GCC, MSVC)
*   CMake 3.22+

### Building

```bash
mkdir build
cd build
cmake ..
make
```

### Running the Renderer

To render a scene, provide the path to the scene file (typically a JSON file describing the scene).

```bash
./caramel <path_to_scene_file>
```

**Example:**
```bash
./caramel ../caramel-scenes/cbox/scene.json
```
This will generate an output image (EXR format) in the current directory, named after the scene file (e.g., `scene.exr`).

### Running Tests

The project includes unit and render tests.

```bash
./caramel_test
```

## Key Files

*   `main.cpp`: Entry point for the standalone renderer. Parses command line arguments and triggers the render loop.
*   `src/render.cpp`: Contains the high-level `render()` function that coordinates scene loading and the main rendering loop.
*   `src/scene_parser.cpp`: logic for parsing scene files (JSON).
*   `src/integrators/path.cpp`: Implementation of the path tracing algorithm.
*   `CMakeLists.txt`: Main build configuration.

## Development Conventions

*   **Code Style:** Follows standard C++ conventions. See `.clang-format` for specific formatting rules.
*   **Logging:** Uses a custom `CRM_ERROR`, `CRM_LOG` etc. macros defined in `include/logger.h`.
*   **Testing:** New features should be accompanied by tests in `test/`. Use `caramel_test` to verify changes.
