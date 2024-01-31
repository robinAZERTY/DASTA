to modify !

# Documentation of BezierCurve.hpp

`BezierCurve.hpp` is a C++ header file that defines a class `BezierPositionMotion`. This class is used to describe a Bezier motion in 4D (3D + heading) using four points and a duration.

## Class `BezierPositionMotion`

### Constructors

- `BezierPositionMotion(const Vector &A, const Vector &B, const Vector &C, const Vector &D, const double &alpha)`: Constructs a `BezierPositionMotion` object with four 4D points and a duration.
- `BezierPositionMotion(const Vector &P0, const Vector &V0, const Vector &P1, const Vector &V1, const double &alpha)`: Constructs a `BezierPositionMotion` object with initial and final position and velocity vectors and a duration.

### Methods

- `void updatePose(const double &t)`: Updates the position and velocity of the motion at a given time `t`.
- `Vector getPos()`: Returns the current position of the motion.
- `Vector getVel()`: Returns the current velocity of the motion.
- `Vector getInitialPos()`: Returns the initial position of the motion.
- `Vector getInitialVel()`: Returns the initial velocity of the motion.
- `Vector getFinalPos()`: Returns the final position of the motion.
- `Vector getFinalVel()`: Returns the final velocity of the motion.

### Private Members

- `Vector A, B, C, D, state`: 4D vectors used to store the four points of the Bezier curve and the current state of the motion.
- `double alpha`: Used to store the duration of the motion.




