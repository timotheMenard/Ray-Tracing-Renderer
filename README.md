# Ray Tracing Renderer with Cat Mesh

This project was completed as part of a university course.

This is a **ray tracer** that renders a 3D scene containing a **cat model** with realistic lighting effects.  
The program creates an image file (`cat.png`) showing the rendered scene.

---

## File Overview

### `ray_tracing_lab.cpp`

This file contains the core functionality of the ray tracer:

- **Vector Class**  
  Implements 3D vectors with operations like addition, subtraction, dot and cross products, and normalisation.

- **Ray Class**  
  Represents a ray with an origin point and direction vector.

- **Intersection Class**  
  Contains information about ray-object intersections, including distance, position, normal, material properties, etc.

- **Geometry Class**  
  An abstract base class for all 3D objects in the scene.

- **Sphere Class**  
  Derived from Geometry, implements sphere objects with intersection calculations.

- **Scene Class**  
  Manages all objects in the scene and handles ray tracing through the scene.

- **BoundingBox Class**  
  Used for optimisation with bounding volume hierarchies.

- **TriangleMesh Class**  
  Implements triangle mesh geometry, including:
  - Loading meshes from OBJ files
  - Building a bounding volume hierarchy (BVH) for efficient ray-triangle intersection
  - Ray-triangle intersection logic

### Utility Functions

- `boxMuller()`  
  Generates normally distributed random numbers.

- `randomCos()`  
  Generates random directions weighted by cosine.

---

### `main.cpp`

This file sets up and runs the ray tracer:

#### Scene Setup

- Creates a **512x512 pixel** image.
- Sets up a scene with a **cat model**.
- Positions a **camera** and **light source**.

#### Ray Tracing Loop

- Uses **anti-aliasing** with multiple samples per pixel (**64 paths**).
- For each pixel, it traces rays through the scene and computes color.

#### Image Output

- Saves the rendered image as **`cat.png`** using the `stb_image_write` library.

---

## Libraries

This project uses the following libraries from the stb single-file public domain libraries by Sean Barrett:

- `stb_image.h`  
  A image loading library that supports multiple formats (PNG, JPEG, BMP, TGA, etc.)

- `stb_image_write.h`  
  A image writing library that can save images in various formats.

---

## Example Renders

Example rendered images are available in the **Renders** folder.
