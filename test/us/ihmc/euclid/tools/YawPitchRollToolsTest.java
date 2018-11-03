package us.ihmc.euclid.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class YawPitchRollToolsTest
{
   public static final double EPSILON = 1.0e-12;
   public static final int ITERATIONS = 1000;

   @Test
   public void testIsZero()
   {
      Random random = new Random(342);
      double yaw, pitch, roll;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         yaw = pitch = roll = 0.0;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         yaw = Double.NaN;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = Double.NaN;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = Double.NaN;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));

         yaw = pitch = roll = 0.0;
         yaw = epsilon;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = epsilon;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = epsilon;
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));

         yaw = pitch = roll = 0.0;
         yaw = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));

         yaw = pitch = roll = 0.0;
         yaw = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isZero(yaw, pitch, roll, epsilon));
      }
   }

   @Test
   public void testIsOrientation2D()
   {
      Random random = new Random(342);
      double yaw, pitch, roll;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         yaw = pitch = roll = 0.0;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         yaw = Double.NaN;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         pitch = Double.NaN;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         yaw = pitch = roll = 0.0;
         roll = Double.NaN;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));

         yaw = EuclidCoreRandomTools.nextDouble(random, 100.0);
         pitch = roll = 0.0;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         pitch = epsilon;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         roll = epsilon;
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));

         pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));

         pitch = roll = 0.0;
         pitch = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
         pitch = roll = 0.0;
         roll = 1.01 * epsilon;
         assertFalse(YawPitchRollTools.isOrientation2D(yaw, pitch, roll, epsilon));
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(5321);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll firstYPR = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll secondYPR = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Quaternion firstQ = new Quaternion(firstYPR);
         Quaternion secondQ = new Quaternion(secondYPR);

         assertEquals(firstQ.distance(secondQ), YawPitchRollTools.distance(firstYPR, secondYPR), EPSILON);
         assertEquals(firstQ.distance(secondQ), YawPitchRollTools.distance(firstYPR.getYaw(), firstYPR.getPitch(), firstYPR.getRoll(), secondYPR.getYaw(),
                                                                           secondYPR.getPitch(), secondYPR.getRoll()),
                      EPSILON);
      }
   }

   @Test
   public void l() throws Exception
   {
      Random random = new Random(24546654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.addTransform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).addTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.addTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).addTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, false);
         new RotationMatrix(ypr).transform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

         try
         {
            YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual, true);
         }
         catch (nota e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual, false);
         new RotationMatrix(ypr).transform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(double yaw, double pitch, double roll, Vector4DReadOnly tupleOriginal, Vector4DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         YawPitchRollTools.transform(ypr.getYaw(), ypr.getPitch(), ypr.getRoll(), tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(YawPitchRollReadOnly yawPitchRoll, Vector4DReadOnly tupleOriginal, Vector4DBasics tupleTransformed)
         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         YawPitchRollTools.transform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }
   }
}
