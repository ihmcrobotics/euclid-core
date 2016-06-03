package us.ihmc.geometry.testingTools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.Matrix3DFeatures;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple.TupleTools;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Tuple4DTools;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class GeometryBasicsTestTools
{
   public static void assertAngleEquals(double expectedAngle, double actualAngle, double epsilon)
   {
      double differenceAngle = Math.abs(expectedAngle - actualAngle);
      differenceAngle = ((differenceAngle + Math.PI) % (2.0 * Math.PI)) - Math.PI;
      try
      {
         assertEquals(0.0, differenceAngle, epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:<" + expectedAngle + "> but was:<" + actualAngle + ">");
      }
   }

   public static void assertYawPitchRollEquals(double[] expectedYawPitchRoll, double[] actualYawPitchRoll, double epsilon)
   {
      try
      {
         assertAngleEquals(expectedYawPitchRoll[0], actualYawPitchRoll[0], epsilon);
         assertAngleEquals(expectedYawPitchRoll[1], actualYawPitchRoll[1], epsilon);
         assertAngleEquals(expectedYawPitchRoll[2], actualYawPitchRoll[2], epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:<" + Arrays.toString(expectedYawPitchRoll) + "> but was:<" + Arrays.toString(actualYawPitchRoll) + ">");
      }
   }

   public static void assertRotationVectorEquals(VectorReadOnly expectedRotationVector, VectorReadOnly actualRotationVector, double epsilon)
   {
      try
      {
         assertAxisAngleEqualsSmart(new AxisAngle(expectedRotationVector), new AxisAngle(actualRotationVector), epsilon);
         // More reliable by going through the axis-angle.
         //         assertAngleEquals(expectedRotationVector.getX(), actualRotationVector.getX(), epsilon);
         //         assertAngleEquals(expectedRotationVector.getY(), actualRotationVector.getY(), epsilon);
         //         assertAngleEquals(expectedRotationVector.getZ(), actualRotationVector.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:" + expectedRotationVector + " but was:" + actualRotationVector);
      }
   }

   public static void assertTupleEquals(TupleReadOnly expected, TupleReadOnly actual, double epsilon)
   {
      assertTupleEquals("", expected, actual, epsilon);
   }

   public static void assertTupleEquals(String message, TupleReadOnly expected, TupleReadOnly actual, double epsilon)
   {
      boolean areEqual = TupleTools.epsilonEquals(expected, actual, epsilon);

      if (message.equals(""))
      {
         message = expected.getClass().getSimpleName() + " is not equal to " + actual.getClass().getSimpleName() + "!";
      }

      if (!areEqual)
      {
         Vector difference = new Vector(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". Difference magnitude = " + difference.length());
      }
   }

   public static void assertTuple2DEquals(Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      assertTuple2DEquals("", expected, actual, epsilon);
   }

   public static void assertTuple2DEquals(String message, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      boolean areEqual = TupleTools.epsilonEquals(expected, actual, epsilon);

      if (message.equals(""))
      {
         message = expected.getClass().getSimpleName() + " is not equal to " + actual.getClass().getSimpleName() + "!";
      }

      if (!areEqual)
      {
         Vector2D difference = new Vector2D(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". Difference magnitude = " + difference.length());
      }
   }

   public static void assertTuple4DEquals(Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      assertTuple4DEquals("", expected, actual, epsilon);
   }

   public static void assertTuple4DEquals(String message, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      boolean areEqual = Tuple4DTools.epsilonEquals(expected, actual, epsilon);

      if (message.equals(""))
      {
         message = expected.getClass().getSimpleName() + " is not equal to " + actual.getClass().getSimpleName() + "!";
      }

      if (!areEqual)
      {
         Vector4D difference = new Vector4D(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". Difference magnitude = " + difference.length());
      }
   }

   public static void assertMatrix3DEquals(Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon)
   {
      assertMatrix3DEquals("", expected, actual, epsilon);
   }

   public static void assertMatrix3DEquals(String message, Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon)
   {
      if (!Matrix3DFeatures.epsilonEquals(expected, actual, epsilon))
      {
         fail(message + " Expected =\n" + expected + "\nActual =\n" + actual);
      }
   }

   /**
    * Verifies whether the given matrix is skew-symmetric.
    *
    * @param matrix     matrix to check for skew-symmetry
    * @param epsilon numerical tolerance
    */
   public static void assertSkewSymmetric(Matrix3DReadOnly matrix, double epsilon)
   {
      if (!Matrix3DFeatures.isMatrixSkewSymmetric(matrix, epsilon))
         fail("The matrix is not skew-symmetric:\n" + matrix);
   }

   public static void assertRotationMatrix(Matrix3DReadOnly matrix, double epsilon)
   {
      if (!Matrix3DFeatures.isRotationMatrix(matrix, epsilon))
         fail("This is not a rotation matrix:\n" + matrix);
   }

   public static void assertIdentity(Matrix3DReadOnly matrix, double epsilon)
   {
      if (!Matrix3DFeatures.isIdentity(matrix, epsilon))
         fail("The matrix is not identity:\n" + matrix);
   }

   public static void assertMatrix3DContainsOnlyNaN(Matrix3DReadOnly matrix)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (!Double.isNaN(matrix.getElement(row, column)))
               fail("The matrix does not contain only NaN:\n" + matrix);
         }
      }
   }

   public static void assertQuaternionEquals(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      try
      {
         assertEquals(expectedQuaternion.getS(), actualQuaternion.getS(), epsilon);
         assertEquals(expectedQuaternion.getX(), actualQuaternion.getX(), epsilon);
         assertEquals(expectedQuaternion.getY(), actualQuaternion.getY(), epsilon);
         assertEquals(expectedQuaternion.getZ(), actualQuaternion.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
      }
   }

   public static void assertQuaternionEqualsSmart(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      try
      {
         double sign = Math.signum(Tuple4DTools.dot(expectedQuaternion, actualQuaternion));
         assertEquals(expectedQuaternion.getS(), sign * actualQuaternion.getS(), epsilon);
         assertEquals(expectedQuaternion.getX(), sign * actualQuaternion.getX(), epsilon);
         assertEquals(expectedQuaternion.getY(), sign * actualQuaternion.getY(), epsilon);
         assertEquals(expectedQuaternion.getZ(), sign * actualQuaternion.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
      }
   }

   public static void assertQuaternionEqualsUsingDifference(QuaternionReadOnly q1, QuaternionReadOnly q2, double epsilon)
   {
      try
      {
         Quaternion qDifference = new Quaternion(q1);
         qDifference.multiplyConjugateOther(q2);
         AxisAngle axisAngle = new AxisAngle();
         axisAngle.set(qDifference);
         assertEquals(0.0, axisAngle.getAngle(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + q1 + ">\n but was:\n<" + q2 + ">");
      }
   }

   public static void assertAxisAngleEquals(AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon)
   {
      try
      {
         assertEquals(expectedAxisAngle.getAngle(), actualAxisAngle.getAngle(), epsilon);
         assertEquals(expectedAxisAngle.getX(), actualAxisAngle.getX(), epsilon);
         assertEquals(expectedAxisAngle.getY(), actualAxisAngle.getY(), epsilon);
         assertEquals(expectedAxisAngle.getZ(), actualAxisAngle.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
      }
   }

   public static void assertAxisAngleEqualsSmart(AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon)
   {
      try
      {
         double expectedX = expectedAxisAngle.getX();
         double expectedY = expectedAxisAngle.getY();
         double expectedZ = expectedAxisAngle.getZ();
         double expectedAngle = expectedAxisAngle.getAngle();
         double actualX = actualAxisAngle.getX();
         double actualY = actualAxisAngle.getY();
         double actualZ = actualAxisAngle.getZ();
         double actualAngle = actualAxisAngle.getAngle();

         if (expectedX * actualX + expectedY * actualY + expectedZ * actualZ < 0.0)
         {
            actualX *= -1.0;
            actualY *= -1.0;
            actualZ *= -1.0;
            actualAngle *= -1.0;
         }

         assertEquals(expectedX, actualX, epsilon);
         assertEquals(expectedY, actualY, epsilon);
         assertEquals(expectedZ, actualZ, epsilon);
         assertAngleEquals(expectedAngle, actualAngle, epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
      }
   }

   public static void assertAxisAngleContainsOnlyNaN(AxisAngleReadOnly axisAngleToAssert)
   {
      assertTrue(Double.isNaN(axisAngleToAssert.getX()));
      assertTrue(Double.isNaN(axisAngleToAssert.getY()));
      assertTrue(Double.isNaN(axisAngleToAssert.getZ()));
      assertTrue(Double.isNaN(axisAngleToAssert.getAngle()));
   }

   public static void assertAxisAngleIsSetToZero(AxisAngleReadOnly axisAngleToAssert)
   {
      assertTrue(axisAngleToAssert.getX() == 1.0);
      assertTrue(axisAngleToAssert.getY() == 0.0);
      assertTrue(axisAngleToAssert.getZ() == 0.0);
      assertTrue(axisAngleToAssert.getAngle() == 0.0);
   }

   public static void assertAxisUnitary(AxisAngleReadOnly axisAngleToAssert, double epsilon)
   {
      if (!AxisAngleTools.isAxisUnitary(axisAngleToAssert, epsilon))
         fail("The axis of the given AxisAngle is not unitary: " + axisAngleToAssert);
   }

   public static void assertQuaternionContainsOnlyNaN(QuaternionReadOnly quaternionToAssert)
   {
      assertTrue(Double.isNaN(quaternionToAssert.getX()));
      assertTrue(Double.isNaN(quaternionToAssert.getY()));
      assertTrue(Double.isNaN(quaternionToAssert.getZ()));
      assertTrue(Double.isNaN(quaternionToAssert.getS()));
   }

   public static void assertQuaternionIsSetToZero(QuaternionReadOnly quaternionToAssert)
   {
      assertTrue(quaternionToAssert.getX() == 0.0);
      assertTrue(quaternionToAssert.getY() == 0.0);
      assertTrue(quaternionToAssert.getZ() == 0.0);
      assertTrue(quaternionToAssert.getS() == 1.0);
   }

   public static void assertQuaternionIsUnitary(QuaternionReadOnly quaternionToAssert, double epsilon)
   {
      if (!Tuple4DTools.isUnitary(quaternionToAssert, epsilon))
         fail("The axis of the given Quaternion is not unitary: " + quaternionToAssert);
   }

   public static void assertTupleContainsOnlyNaN(TupleReadOnly tupleToAssert)
   {
      assertTrue(Double.isNaN(tupleToAssert.getX()));
      assertTrue(Double.isNaN(tupleToAssert.getY()));
      assertTrue(Double.isNaN(tupleToAssert.getZ()));
   }

   public static void assertTupleIsSetToZero(TupleReadOnly tupleToAssert)
   {
      assertTrue(tupleToAssert.getX() == 0.0);
      assertTrue(tupleToAssert.getY() == 0.0);
      assertTrue(tupleToAssert.getZ() == 0.0);
   }

   public static void assertRigidBodyTransformEquals(RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }

   public static void assertQuaternionBasedTransformEquals(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }

   public static void assertQuaternionBasedTransformEqualsSmart(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      try
      {
         assertTupleEquals(expected.getTranslationVector(), actual.getTranslationVector(), epsilon);
         assertQuaternionEqualsSmart(expected.getQuaternion(), actual.getQuaternion(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }

   public static void assertAffineTransformEquals(AffineTransform expected, AffineTransform actual, double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }
}
