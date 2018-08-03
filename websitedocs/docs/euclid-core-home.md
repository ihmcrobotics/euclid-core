---
id: euclid-core-home
title: Euclid-Core
sidebar_label: Introduction
---
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

See our [github repository](https://github.com/ihmcrobotics/euclid-core).