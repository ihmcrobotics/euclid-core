package us.ihmc.geometry.testingTools;

import java.util.Random;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngle32;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple.Point;
import us.ihmc.geometry.tuple.Point32;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.Vector32;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple2D.Point2D32;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.Vector2D32;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Quaternion32;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public abstract class GeometryBasicsRandomTools
{
   public static double[] generateRandomYawPitchRoll(Random random)
   {
      return generateRandomYawPitchRoll(random, Math.PI, YawPitchRollConversion.MAX_PITCH_ANGLE, Math.PI);
   }

   public static double[] generateRandomYawPitchRoll(Random random, double minMaxYaw, double minMaxPitch, double minMaxRoll)
   {
      double yaw = generateRandomDouble(random, minMaxYaw);
      double pitch = generateRandomDouble(random, minMaxPitch);
      double roll = generateRandomDouble(random, minMaxRoll);
      double[] yawPitchRoll = {yaw, pitch, roll};
      return yawPitchRoll;
   }

   public static Vector generateRandomRotationVector(Random random)
   {
      return generateRandomRotationVector(random, Math.PI);
   }

   public static Vector generateRandomRotationVector(Random random, double minMaxAngleRange)
   {
      Vector rotationVector = new Vector();
      AxisAngle axisAngle = generateRandomAxisAngle(random, minMaxAngleRange);
      axisAngle.getRotationVector(rotationVector);
      return rotationVector;
   }

   public static AxisAngle generateRandomAxisAngle(Random random)
   {
      AxisAngle axisAngle = new AxisAngle();
      randomizeAxisAngle(random, axisAngle);
      return axisAngle;
   }

   public static AxisAngle generateRandomAxisAngle(Random random, double minMaxAngleRange)
   {
      AxisAngle axisAngle = new AxisAngle();
      randomizeAxisAngle(random, minMaxAngleRange, axisAngle);
      return axisAngle;
   }

   public static AxisAngle32 generateRandomAxisAngle32(Random random)
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      randomizeAxisAngle(random, axisAngle);
      return axisAngle;
   }

   public static AxisAngle32 generateRandomAxisAngle32(Random random, double minMaxAngleRange)
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      randomizeAxisAngle(random, minMaxAngleRange, axisAngle);
      return axisAngle;
   }

   public static Matrix3D generateRandomDiagonalMatrix3D(Random random)
   {
      return generateRandomMatrix3D(random, 1.0);
   }

   public static Matrix3D generateRandomDiagonalMatrix3D(Random random, double elementMagnitude)
   {
      return generateRandomMatrix3D(random, -0.5 * elementMagnitude, 0.5 * elementMagnitude);
   }

   public static Matrix3D generateRandomDiagonalMatrix3D(Random random, double elementMin, double elementMax)
   {
      Matrix3D matrix3D = new Matrix3D();
      for (int i = 0; i < 3; i++)
         matrix3D.setElement(i, i, generateRandomDouble(random, elementMin, elementMax));
      return matrix3D;
   }

   public static double generateRandomDouble(Random random)
   {
      return generateRandomDouble(random, 1.0);
   }

   public static double generateRandomDouble(Random random, double minMax)
   {
      return generateRandomDouble(random, -minMax, minMax);
   }

   public static double generateRandomDouble(Random random, double minValue, double maxValue)
   {
      if (minValue > maxValue)
         throw new RuntimeException("Min is greater than max: min = " + minValue + ", max = " + maxValue);

      return minValue + random.nextDouble() * (maxValue - minValue);
   }

   public static Matrix3D generateRandomMatrix3D(Random random)
   {
      return generateRandomMatrix3D(random, 1.0);
   }

   public static Matrix3D generateRandomMatrix3D(Random random, double elementMagnitude)
   {
      return generateRandomMatrix3D(random, -0.5 * elementMagnitude, 0.5 * elementMagnitude);
   }

   public static Matrix3D generateRandomMatrix3D(Random random, double elementMin, double elementMax)
   {
      Matrix3D matrix3D = new Matrix3D();
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            matrix3D.setElement(row, column, generateRandomDouble(random, elementMin, elementMax));
         }
      }
      return matrix3D;
   }

   public static Quaternion generateRandomQuaternion(Random random)
   {
      return new Quaternion(generateRandomAxisAngle(random));
   }

   public static Quaternion generateRandomQuaternion(Random random, double minMaxAngleRange)
   {
      return new Quaternion(generateRandomAxisAngle(random, minMaxAngleRange));
   }

   public static Quaternion32 generateRandomQuaternion32(Random random)
   {
      return new Quaternion32(generateRandomAxisAngle(random));
   }

   public static Quaternion32 generateRandomQuaternion32(Random random, double minMaxAngleRange)
   {
      return new Quaternion32(generateRandomAxisAngle(random, minMaxAngleRange));
   }

   public static RigidBodyTransform generateRandomRigidBodyTransform(Random random)
   {
      return new RigidBodyTransform(generateRandomAxisAngle(random), generateRandomVector(random));
   }

   public static QuaternionBasedTransform generateRandomQuaternionBasedTransform(Random random)
   {
      return new QuaternionBasedTransform(generateRandomQuaternion(random), generateRandomVector(random));
   }

   public static AffineTransform generateRandomAffineTransform(Random random)
   {
      return new AffineTransform(generateRandomRotationScaleMatrix(random, 10.0), generateRandomVector(random));
   }

   public static RotationMatrix generateRandomRotationMatrix(Random random)
   {
      return generateRandomRotationMatrix(random, Math.PI);
   }

   public static RotationMatrix generateRandomRotationMatrix(Random random, double minMaxAngleRange)
   {
      AxisAngle randomRotation = generateRandomAxisAngle(random, minMaxAngleRange);
      return new RotationMatrix(randomRotation);
   }

   public static RotationScaleMatrix generateRandomRotationScaleMatrix(Random random, double maxScale)
   {
      return generateRandomRotationScaleMatrix(random, Math.PI, maxScale);
   }

   public static RotationScaleMatrix generateRandomRotationScaleMatrix(Random random, double minMaxAngleRange, double maxScale)
   {
      AxisAngle randomRotation = generateRandomAxisAngle(random, minMaxAngleRange);
      Vector randomScales = generateRandomVector(random, new Vector(0.0, 0.0, 0.0), new Vector(maxScale, maxScale, maxScale));
      return new RotationScaleMatrix(randomRotation, randomScales);
   }

   public static Vector generateRandomVector(Random random)
   {
      Vector vector = new Vector();
      randomizeTuple(random, vector);
      return vector;
   }

   public static Point generateRandomPoint(Random random)
   {
      Point point = new Point();
      randomizeTuple(random, point);
      return point;
   }

   public static Vector generateRandomVectorWithFixedLength(Random random, double length)
   {
      Vector vector = generateRandomVector(random);
      vector.normalize();
      vector.scale(length);
      return vector;
   }

   public static Vector generateRandomVector(Random random, TupleReadOnly minMax)
   {
      Vector vector = new Vector();
      randomizeTuple(random, minMax, vector);
      return vector;
   }

   public static Vector generateRandomVector(Random random, TupleReadOnly min, TupleReadOnly max)
   {
      Vector vector = new Vector();
      randomizeTuple(random, min, max, vector);
      return vector;
   }

   public static Vector generateRandomVector(Random random, double min, double max)
   {
      Vector vector = new Vector();
      randomizeTuple(random, new Point(min, min, min), new Point(max, max, max), vector);
      return vector;
   }

   public static Vector2D generateRandomVector2D(Random random)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, vector);
      return vector;
   }

   public static Point2D generateRandomPoint2D(Random random)
   {
      Point2D point = new Point2D();
      randomizeTuple2D(random, point);
      return point;
   }
   
   public static Vector2D generateRandomVector2DWithFixedLength(Random random, double length)
   {
      Vector2D vector = generateRandomVector2D(random);
      vector.normalize();
      vector.scale(length);
      return vector;
   }

   public static Vector2D generateRandomVector2D(Random random, Tuple2DReadOnly minMax)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, minMax, vector);
      return vector;
   }

   public static Vector2D generateRandomVector2D(Random random, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, min, max, vector);
      return vector;
   }

   public static Vector4D generateRandomVector4D(Random random)
   {
      Vector4D vector = new Vector4D();
      for (int i = 0; i < 4; i++)
         vector.set(i, random.nextDouble());
      return vector;
   }

   public static Vector32 generateRandomVector32(Random random)
   {
      Vector32 vector = new Vector32();
      randomizeTuple(random, vector);
      return vector;
   }

   public static Point32 generateRandomPoint32(Random random)
   {
      Point32 point = new Point32();
      randomizeTuple(random, point);
      return point;
   }

   public static Vector2D32 generateRandomVector2D32(Random random)
   {
      Vector2D32 vector = new Vector2D32();
      randomizeTuple2D(random, vector);
      return vector;
   }

   public static Point2D32 generateRandomPoint2D32(Random random)
   {
      Point2D32 point = new Point2D32();
      randomizeTuple2D(random, point);
      return point;
   }

   public static void randomizeAxisAngle(Random random, AxisAngleBasics<?> axisAngleToRandomize)
   {
      randomizeAxisAngle(random, Math.PI, axisAngleToRandomize);
   }

   public static void randomizeAxisAngle(Random random, double minMaxAngleRange, AxisAngleBasics<?> axisAngleToRandomize)
   {
      // Generate uniformly random point on unit sphere (based on http://mathworld.wolfram.com/SpherePointPicking.html )
      double height = 2.0 * random.nextDouble() - 1.0;
      double angle = 2.0 * minMaxAngleRange * random.nextDouble() - minMaxAngleRange;
      double radius = Math.sqrt(1.0 - height * height);
      axisAngleToRandomize.set(radius * Math.cos(angle), radius * Math.sin(angle), height, angle);
   }

   public static void randomizeTuple(Random random, TupleBasics tupleToRandomize)
   {
      randomizeTuple(random, new Point(1.0, 1.0, 1.0), tupleToRandomize);
   }

   public static void randomizeTuple(Random random, TupleReadOnly minMax, TupleBasics tupleToRandomize)
   {
      for (int i = 0; i < 3; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, minMax.get(i)));
   }

   public static void randomizeTuple(Random random, TupleReadOnly min, TupleReadOnly max, TupleBasics tupleToRandomize)
   {
      for (int i = 0; i < 3; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, min.get(i), max.get(i)));
   }

   public static void randomizeTuple2D(Random random, Tuple2DBasics tupleToRandomize)
   {
      randomizeTuple2D(random, new Point2D(1.0, 1.0), tupleToRandomize);
   }

   public static void randomizeTuple2D(Random random, Tuple2DReadOnly minMax, Tuple2DBasics tupleToRandomize)
   {
      for (int i = 0; i < 2; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, minMax.get(i)));
   }

   public static void randomizeTuple2D(Random random, Tuple2DReadOnly min, Tuple2DReadOnly max, Tuple2DBasics tupleToRandomize)
   {
      for (int i = 0; i < 2; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, min.get(i), max.get(i)));
   }
}
