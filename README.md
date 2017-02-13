# IHMC Geometry Basics

## Minutiae

### Tested Platforms
We test all of our software on OS X 10.11 El Capitan, Windows 7/8/10, and Ubuntu 14.04 LTS Desktop and Server.

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

### Compatibility
This library is compatible with Java 8+.

### Dependency
This library sources depends on the matrix library EJML [here](http://ejml.org/), while the tests also on JUnit and PIT mutation testing library [here](http://pitest.org/).

## Briefly
IHMC Geometry Basics is a library that defines the common tools useful for general geometric applications.
The library is born from the need of having a base geometry library that is well tested, flexible, and meant for real-time environment where
garbage generation is not allowed.

## Content
This library includes the following:
- Points in 2 and 3 dimensions: Point2D, Point3D, and their single-float precision defined counterparts: Point2D32, Point3D32.
- Vectors in 2, 3, and 4 dimensions: Vector2D, Vector3D, Vector4D, and their single-float precision defined counterparts: Vector2D32, Vector3D32, Vector4D32.
- The definition of several representations of 3D orientations:
 -- as a 3-by-3 rotation matrix: RotationMatrix.
 -- as a quaternion: Quaternion, Quaternion32.
 -- as an axis-angle: AxisAngle, AxisAngle32.
 -- it also provides tools to convert any rotation definition to a rotation vector, also called Euler vector, and to yaw-pitch-roll angles, also called Euler angles.
- A set of tools to easily convert from one orientation definition to another: AxisAngleConversion, QuaternionConversion, RotationMatrixConversion, RotationVectorConversion, and YawPitchRollConversion.
- A general 3-by-3 matrix: Matrix3D.
- A 3-by-3 matrix for rotating and scaling geometry objects: RotationScaleMatrix.
- Geometric transforms:
 -- A 4-by-4 homogeneous matrix for rotating and translating geometry objects: RigidBodyTransform.
 -- The more concise equivalent of the RigidBodyTransform using a quaternion instead of a 3-by-3 rotation matrix: QuaternionBasedTransform.
 -- A 4-by-4 homogeneous matrix for scaling, rotation, and translating geometry objects: AffineTransform.
