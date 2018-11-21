# Euclid Core
# !!! Euclid-core is being merged with [Euclid](https://github.com/ihmcrobotics/euclid) startign at Euclid 0.11.0+. Only minor bugfixes will be carried on this repo. !!!

## Minutiae

### Tested Platforms
We test all of our software on OS X 10.11 El Capitan, Windows 7/8/10, and Ubuntu 14.04 LTS Desktop and Server.

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Test and documentation coverage
We have put our best effort into documenting and testing the entire library. Test coverage is above 90%. 

### Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

### Compatibility
This library is compatible with Java 8+.

### Dependency
This library sources depends on the matrix library EJML [here](http://ejml.org/), while the tests also on JUnit and PIT mutation testing library [here](http://pitest.org/).

## What is Euclid Core?
Euclid Core is a library that defines the common tools useful for general geometric applications and vector math.
The library is born from the need of having a base library for geometric applications that is well tested, flexible, and meant for real-time environment where
garbage generation is not allowed.
This library is meant to be the core of several libraries that are still in development at IHMC and will be released progressively in the near future starting
with the Euclid library that defines generic 2D and 3D geometry object such as lines, planes, and polygons, alongside with 
3D shapes such boxes, cylinders, and polytopes.

## How does Euclid Core work?
As we intend to keep Euclid Core at the low-level of third party software, the possibilities of how to use it are quite numerous.
The main idea is to create and operate on the following variety of possible objects:
- AxisAngle, AxisAngle32,
- Matrix3D, RotationMatrix, RotationScaleMatrix,
- RigidBodyTransform, QuaternionBasedTransform, AffineTransform,
- Point2D, Point2D32, Vector2D, Vector2D32,
- Point3D, Point3D32, Vector3D, Vector3D32,
- Quaternion, Quaternion32, Vector4D, Vector4D32.

Each of these classes is fully documented providing the use-case.
Besides the classes mentioned above, one should take look at the list of conversion utilities to help you convert a orientation definition into another one:
- AxisAngleConversion, QuaternionConversion, RotationMatrixConversion, RotationVectorConversion, YawPitchRollConversion.

## Who would use Euclid Core?
Any software developer manipulating geometry objects or dealing with 2D or 3D graphical UI is susceptible to this library as the base for doing the vector math.

## What is the goal of Euclid Core?
The goal for Euclid Core is to become the most flexible, easy to use, and fast library for vector math, so it results in great increase in development productivity. 
However, this library is intended to remain a very low-level library easy to depend on and to build on top of it.
For instance, no shapes such as boxes or cylinders will be integrated to this library.

## How can I contribute to Euclid Core?
Please read [CONTRIBUTING.md](https://github.com/ihmcrobotics/euclid-core/blob/develop/CONTRIBUTING.md).

## Content
This library includes the following:
- Points in 2 and 3 dimensions: Point2D, Point3D, and their single-float precision defined counterparts: Point2D32, Point3D32.
- Vectors in 2, 3, and 4 dimensions: Vector2D, Vector3D, Vector4D, and their single-float precision defined counterparts: Vector2D32, Vector3D32, Vector4D32.
- The definition of several representations of 3D orientations:
	- as a 3-by-3 rotation matrix: RotationMatrix.
	- as a quaternion: Quaternion, Quaternion32.
	- as an axis-angle: AxisAngle, AxisAngle32.
	- it also provides tools to convert any rotation definition to a rotation vector, also called Euler vector, and to yaw-pitch-roll angles, also called Euler angles.
- A set of tools to easily convert from one orientation definition to another: AxisAngleConversion, QuaternionConversion, RotationMatrixConversion, RotationVectorConversion, and YawPitchRollConversion.
- A general 3-by-3 matrix: Matrix3D.
- A 3-by-3 matrix for rotating and scaling geometry objects: RotationScaleMatrix.
- Geometric transforms:
	 - A 4-by-4 homogeneous matrix for rotating and translating geometry objects: RigidBodyTransform.
	 - The more concise equivalent of the RigidBodyTransform using a quaternion instead of a 3-by-3 rotation matrix: QuaternionBasedTransform.
	 - A 4-by-4 homogeneous matrix for scaling, rotation, and translating geometry objects: AffineTransform.

## Using Euclid Core from .jar releases with Maven/Gradle
The releases .jars for Euclid Core are hosted on Bintray.
You can browse the IHMC release packages at https://bintray.com/ihmcrobotics/maven-release.
Instructions for adding the Maven repository and identifying the artifacts can also be found on Bintray for each package.

At a minimum, you will need to have the following repository declared in your build script to use the Euclid Core .jars:

```gradle
repositories {
   maven {
      url  "http://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
   }

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}
```

Here is an example for adding the dependency to Euclid Core using your build script:
```gradle
dependencies {
   compile group: 'us.ihmc', name: 'euclid-core', version: '0.2'
}
```
