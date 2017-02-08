package us.ihmc.geometry.transform;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Vector4D;

public class RigidBodyTransformTest
{
   private static final double EPS = 1.0e-10;
   public static final int NUMBER_OF_ITERATIONS = 100;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345L);

      { // Test empty constructor
         RigidBodyTransform transform = new RigidBodyTransform();
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               if (row == column)
                  assertTrue(transform.getElement(row, column) == 1.0);
               else
                  assertTrue(transform.getElement(row, column) == 0.0);
            }
         }
      }

      { // Test RigidBodyTransform(RigidBodyTransform other)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform(expected);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test RigidBodyTransform(QuaternionBasedTransform quaternionBasedTransform)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform(new QuaternionBasedTransform(expected));
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertEquals(expected.getElement(row, column), actual.getElement(row, column), EPS);
            }
         }
      }

      { // Test RigidBodyTransform(DenseMatrix64F matrix)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RigidBodyTransform actual = new RigidBodyTransform(denseMatrix);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test RigidBodyTransform(Matrix3DReadOnly rotationMatrix, TupleReadOnly translation)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);

         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(denseMatrix);
         Vector3D vector = new Vector3D();
         vector.set(0, 3, denseMatrix);

         RigidBodyTransform actual = new RigidBodyTransform(rotationMatrix, vector);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test RigidBodyTransform(QuaternionReadOnly quaternion, TupleReadOnly translation)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);

         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(denseMatrix);
         Vector3D vector = new Vector3D();
         vector.set(0, 3, denseMatrix);

         RigidBodyTransform actual = new RigidBodyTransform(new Quaternion(rotationMatrix), vector);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test RigidBodyTransform(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);

         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(denseMatrix);
         Vector3D vector = new Vector3D();
         vector.set(0, 3, denseMatrix);

         RigidBodyTransform actual = new RigidBodyTransform(new AxisAngle(rotationMatrix), vector);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test RigidBodyTransform(double[] transformArray)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         double[] transformArray = new double[16];
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               transformArray[4 * row + column] = expected.getElement(row, column);
            }
         }

         RigidBodyTransform actual = new RigidBodyTransform(transformArray);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test RigidBodyTransform(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);

         double m00 = expected.getM00();
         double m01 = expected.getM01();
         double m02 = expected.getM02();
         double m03 = expected.getM03();
         double m10 = expected.getM10();
         double m11 = expected.getM11();
         double m12 = expected.getM12();
         double m13 = expected.getM13();
         double m20 = expected.getM20();
         double m21 = expected.getM21();
         double m22 = expected.getM22();
         double m23 = expected.getM23();

         RigidBodyTransform actual = new RigidBodyTransform(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }
   }

   @Test
   public void testResetRotation() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      transform.setRotationToZero();

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column == 3)
               assertTrue(transform.getElement(row, column) == original.getElement(row, column));
            else if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testResetTranslation() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      transform.setTranslationToZero();

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column < 3)
               assertTrue(transform.getElement(row, column) == original.getElement(row, column));
            else if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testNormalizeRotationPart() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      double corruptionFactor = 0.1;
      double m00 = original.getM00() + corruptionFactor * random.nextDouble();
      double m01 = original.getM01() + corruptionFactor * random.nextDouble();
      double m02 = original.getM02() + corruptionFactor * random.nextDouble();
      double m10 = original.getM10() + corruptionFactor * random.nextDouble();
      double m11 = original.getM11() + corruptionFactor * random.nextDouble();
      double m12 = original.getM12() + corruptionFactor * random.nextDouble();
      double m20 = original.getM20() + corruptionFactor * random.nextDouble();
      double m21 = original.getM21() + corruptionFactor * random.nextDouble();
      double m22 = original.getM22() + corruptionFactor * random.nextDouble();
      transform.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      transform.normalizeRotationPart();

      Matrix3D rotation = new Matrix3D();
      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      transform.getRotation(rotation);

      // Test that each row & column vectors are unit-length
      for (int j = 0; j < 3; j++)
      {
         rotation.getRow(j, vector1);
         assertEquals(1.0, vector1.length(), EPS);

         rotation.getColumn(j, vector1);
         assertEquals(1.0, vector1.length(), EPS);
      }

      // Test that each pair of rows and each pair of columns are orthogonal
      for (int j = 0; j < 3; j++)
      {
         rotation.getRow(j, vector1);
         rotation.getRow((j + 1) % 3, vector2);
         assertEquals(0.0, vector1.dot(vector2), EPS);

         rotation.getColumn(j, vector1);
         rotation.getColumn((j + 1) % 3, vector2);
         assertEquals(0.0, vector1.dot(vector2), EPS);
      }
   }

   @Test
   public void testDeterminantRotationPart() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      double corruptionFactor = 0.1;
      double m00 = original.getM00() * corruptionFactor;
      double m01 = original.getM01() * corruptionFactor;
      double m02 = original.getM02() * corruptionFactor;
      double m10 = original.getM10() * corruptionFactor;
      double m11 = original.getM11() * corruptionFactor;
      double m12 = original.getM12() * corruptionFactor;
      double m20 = original.getM20() * corruptionFactor;
      double m21 = original.getM21() * corruptionFactor;
      double m22 = original.getM22() * corruptionFactor;
      transform.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      assertTrue(transform.determinantRotationPart() < corruptionFactor);
      transform.normalizeRotationPart();
      assertEquals(1.0, transform.determinantRotationPart(), EPS);
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(34534L);
      RigidBodyTransform expected = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform actual = new RigidBodyTransform();

      { // Test set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         actual.setIdentity();
         double m00 = expected.getM00();
         double m01 = expected.getM01();
         double m02 = expected.getM02();
         double m03 = expected.getM03();
         double m10 = expected.getM10();
         double m11 = expected.getM11();
         double m12 = expected.getM12();
         double m13 = expected.getM13();
         double m20 = expected.getM20();
         double m21 = expected.getM21();
         double m22 = expected.getM22();
         double m23 = expected.getM23();
         actual.set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test setUnsafe(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         actual.setIdentity();
         double m00 = expected.getM00();
         double m01 = expected.getM01();
         double m02 = expected.getM02();
         double m03 = expected.getM03();
         double m10 = expected.getM10();
         double m11 = expected.getM11();
         double m12 = expected.getM12();
         double m13 = expected.getM13();
         double m20 = expected.getM20();
         double m21 = expected.getM21();
         double m22 = expected.getM22();
         double m23 = expected.getM23();
         actual.setUnsafe(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test set(RigidBodyTransform other)
         actual.setIdentity();
         actual.set(expected);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test set(QuaternionBasedTransform quaternionBasedTransform)
         actual.setIdentity();
         actual.set(new QuaternionBasedTransform(expected));
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test set(DenseMatrix64F matrix)
         actual.setIdentity();
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test set(DenseMatrix64F matrix, int startRow, int startColumn)
         actual.setIdentity();
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4 + startRow, 4 + startColumn);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row + startRow, column + startColumn, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix, startRow, startColumn);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test set(double[] transformArray)
         actual.setIdentity();
         double[] transformArray = new double[16];
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               transformArray[4 * row + column] = expected.getElement(row, column);
            }
         }
         actual.set(transformArray);
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test set(Matrix3DReadOnly rotationMatrix, TupleReadOnly translation)
         RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Vector3D translation = GeometryBasicsRandomTools.generateRandomVector3D(random);
         actual.set((Matrix3DReadOnly<?>) rotationMatrix, translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.get(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(RotationMatrix rotationMatrix, TupleReadOnly translation)
         RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Vector3D translation = GeometryBasicsRandomTools.generateRandomVector3D(random);
         actual.set(rotationMatrix, translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.get(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(RotationScaleMatrix rotationMatrix, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         Vector3D translation = GeometryBasicsRandomTools.generateRandomVector3D(random);
         actual.set(rotationScaleMatrix, translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getRotationMatrix().getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.get(row) == actual.getElement(row, 3));
         }
      }
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertFalse(transform.getElement(row, column) == 1.0);
            else
               assertFalse(transform.getElement(row, column) == 0.0);
         }
      }

      transform.setIdentity();

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertFalse(transform.getElement(row, column) == 1.0);
            else
               assertFalse(transform.getElement(row, column) == 0.0);
         }
      }

      transform.setToZero();

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertFalse(Double.isNaN(transform.getElement(row, column)));
         }
      }

      transform.setToNaN();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertTrue(Double.isNaN(transform.getElement(row, column)));
         }
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertFalse(transform.containsNaN());
      transform.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      assertTrue(transform.containsNaN());
   }

   @Test
   public void testSetRotation() throws Exception
   {
      Random random = new Random(2342L);
      RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      Vector3D translation = new Vector3D();

      { // Test setRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);

         double m00 = rotationMatrix.getM00();
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m10 = rotationMatrix.getM10();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();
         double m20 = rotationMatrix.getM20();
         double m21 = rotationMatrix.getM21();
         double m22 = rotationMatrix.getM22();
         transform.setRotation(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setRotationUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);

         double m00 = rotationMatrix.getM00();
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m10 = rotationMatrix.getM10();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();
         double m20 = rotationMatrix.getM20();
         double m21 = rotationMatrix.getM21();
         double m22 = rotationMatrix.getM22();
         transform.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setRotation(AxisAngleReadOnly axisAngle)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         AxisAngle axisAngle = new AxisAngle(rotationMatrix);
         transform.setRotation(axisAngle);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setRotation(DenseMatrix64F matrix)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
         rotationMatrix.get(denseMatrix);
         transform.setRotation(denseMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setRotation(QuaternionReadOnly quaternion)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         Quaternion quaternion = new Quaternion(rotationMatrix);
         transform.setRotation(quaternion);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setRotation(Matrix3DReadOnly rotationMatrix)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         transform.setRotation(rotationMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }
   }

   @Test
   public void testSetRotationYawPitchRoll() throws Exception
   {
      Random random = new Random(234L);
      RotationMatrix expectedRotation = new RotationMatrix();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      double[] yawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);

      { // Test setRotationYawPitchRoll(double yaw, double pitch, double roll)
         expectedRotation.setYawPitchRoll(yawPitchRoll);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Vector3D translation = new Vector3D();
         actualTransform.getTranslation(translation);
         actualTransform.setRotationYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(translation.get(row) == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationYawPitchRollAndZeroTranslation(double yaw, double pitch, double roll)
         expectedRotation.setYawPitchRoll(yawPitchRoll);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         actualTransform.setRotationYawPitchRollAndZeroTranslation(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(0.0 == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationYaw(double yaw)
         expectedRotation.setToYawMatrix(yawPitchRoll[0]);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Vector3D translation = new Vector3D();
         actualTransform.getTranslation(translation);
         actualTransform.setRotationYaw(yawPitchRoll[0]);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(translation.get(row) == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationPitch(double pitch)
         expectedRotation.setToPitchMatrix(yawPitchRoll[1]);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Vector3D translation = new Vector3D();
         actualTransform.getTranslation(translation);
         actualTransform.setRotationPitch(yawPitchRoll[1]);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(translation.get(row) == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationRoll(double roll)
         expectedRotation.setToRollMatrix(yawPitchRoll[2]);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Vector3D translation = new Vector3D();
         actualTransform.getTranslation(translation);
         actualTransform.setRotationRoll(yawPitchRoll[2]);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(translation.get(row) == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationYawAndZeroTranslation(double yaw)
         expectedRotation.setToYawMatrix(yawPitchRoll[0]);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         actualTransform.setRotationYawAndZeroTranslation(yawPitchRoll[0]);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(0.0 == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationPitchAndZeroTranslation(double pitch)
         expectedRotation.setToPitchMatrix(yawPitchRoll[1]);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         actualTransform.setRotationPitchAndZeroTranslation(yawPitchRoll[1]);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(0.0 == actualTransform.getElement(row, column));
         }
      }

      { // Test setRotationRollAndZeroTranslation(double roll)
         expectedRotation.setToRollMatrix(yawPitchRoll[2]);
         actualTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         actualTransform.setRotationRollAndZeroTranslation(yawPitchRoll[2]);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertEquals(expectedRotation.getElement(row, column), actualTransform.getElement(row, column), EPS);
            }
         }

         for (int row = 0; row < 3; row++)
         {
            int column = 3;
            assertTrue(0.0 == actualTransform.getElement(row, column));
         }
      }
   }

   @Test
   public void testSetRotationAndZeroTranslation() throws Exception
   {
      Random random = new Random(2342L);
      RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

      { // Test setRotationAndZeroTranslation(AxisAngleReadOnly axisAngle)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         AxisAngle axisAngle = new AxisAngle(rotationMatrix);
         transform.setRotationAndZeroTranslation(axisAngle);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(transform.getElement(row, 3) == 0.0);
         }
      }

      { // Test setRotationAndZeroTranslation(DenseMatrix64F matrix)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
         rotationMatrix.get(denseMatrix);
         transform.setRotationAndZeroTranslation(denseMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(transform.getElement(row, 3) == 0.0);
         }
      }

      { // Test setRotationAndZeroTranslation(QuaternionReadOnly quaternion)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Quaternion quaternion = new Quaternion(rotationMatrix);
         transform.setRotationAndZeroTranslation(quaternion);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
            assertTrue(transform.getElement(row, 3) == 0.0);
         }
      }

      { // Test setRotationAndZeroTranslation(Matrix3DReadOnly rotationMatrix)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.setRotationAndZeroTranslation(rotationMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(transform.getElement(row, 3) == 0.0);
         }
      }
   }

   @Test
   public void testRotationEuler() throws Exception
   {
      Random random = new Random(42353L);
      RotationMatrix rotationMatrix = new RotationMatrix();
      Vector3D translation = new Vector3D();

      { // Test setRotationEuler(VectorReadOnly eulerAngles)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         Vector3D eulerAngles = GeometryBasicsRandomTools.generateRandomRotationVector(random);
         rotationMatrix.setEuler(eulerAngles);
         transform.setRotationEuler(eulerAngles);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setRotationEulerAndZeroTranslation(VectorReadOnly eulerAngles)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         Vector3D eulerAngles = GeometryBasicsRandomTools.generateRandomRotationVector(random);
         rotationMatrix.setEuler(eulerAngles);
         transform.setRotationEulerAndZeroTranslation(eulerAngles);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(transform.getElement(row, 3) == 0.0);
         }
      }

      { // Test setRotationEuler(double rotX, double rotY, double rotZ)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getTranslation(translation);
         double rotX = Math.PI * random.nextDouble();
         double rotY = Math.PI * random.nextDouble();
         double rotZ = Math.PI * random.nextDouble();
         rotationMatrix.setEuler(rotX, rotY, rotZ);
         transform.setRotationEuler(rotX, rotY, rotZ);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(243L);
      RotationMatrix rotationMatrix = new RotationMatrix();
      Vector3D translation = new Vector3D();

      { // Test setTranslation(TupleReadOnly translation)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getRotation(rotationMatrix);
         transform.setTranslation(translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setTranslation(double x, double y, double z)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getRotation(rotationMatrix);
         transform.setTranslation(translation.getX(), translation.getY(), translation.getZ());
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }

      { // Test setTranslationAndIdentityRotation(TupleReadOnly translation)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.setTranslationAndIdentityRotation(translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               if (row == column)
                  assertTrue(transform.getElement(row, column) == 1.0);
               else
                  assertTrue(transform.getElement(row, column) == 0.0);
            }
            assertTrue(translation.get(row) == transform.getElement(row, 3));
         }
      }
   }

   @Test
   public void testGetRotation() throws Exception
   {
      Random random = new Random(2345L);
      RotationMatrix rotationMatrix = new RotationMatrix();

      { // Test getRotation(Matrix3DBasics rotationMatrixToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getRotation((Matrix3DBasics<?>) rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(RotationMatrix rotationMatrixToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         transform.getRotation(rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(RotationScaleMatrix rotationMatrixToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         transform.getRotation(rotationScaleMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(DenseMatrix64F matrixToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
         transform.getRotation(denseMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(QuaternionBasics quaternionToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Quaternion quaternion = new Quaternion();
         transform.getRotation(quaternion);
         rotationMatrix.set(quaternion);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         AxisAngle axisAngle = new AxisAngle();
         transform.getRotation(axisAngle);
         rotationMatrix.set(axisAngle);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
      }
   }

   @Test
   public void testGetTranslation() throws Exception
   {
      Random random = new Random(2345L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      Vector3D translation = new Vector3D();
      transform.getTranslation(translation);
      for (int row = 0; row < 3; row++)
         assertTrue(translation.get(row) == transform.getElement(row, 3));
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(2342L);

      { // Test get(DenseMatrix64F matrixToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         DenseMatrix64F denseMatrix = RandomMatrices.createRandom(4, 4, random);
         transform.get(denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }

      { // Test get(DenseMatrix64F matrixToPack, int startRow, int startColumn)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DenseMatrix64F denseMatrix = RandomMatrices.createRandom(4 + startRow, 4 + startColumn, random);
         transform.get(startRow, startColumn, denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row + startRow, column + startColumn) == transform.getElement(row, column));
      }

      { // Test get(double[] transformArrayToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         double[] transformArray = new double[16];
         transform.get(transformArray);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(transformArray[4 * row + column] == transform.getElement(row, column));
      }

      { // Test get(QuaternionBasics quaternionToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Quaternion expectedQuaternion = new Quaternion();
         Quaternion actualQuaternion = new Quaternion();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedQuaternion);
         transform.getTranslation(expectedTranslation);
         transform.get(actualQuaternion, actualTranslation);
         assertEquals(expectedQuaternion, actualQuaternion);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(Matrix3DBasics rotationMarixToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Matrix3D expectedMatrix = new Matrix3D();
         Matrix3D actualMatrix = new Matrix3D();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(RotationMatrix rotationMarixToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RotationMatrix expectedMatrix = new RotationMatrix();
         RotationMatrix actualMatrix = new RotationMatrix();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(RotationScaleMatrix rotationMarixToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RotationScaleMatrix expectedMatrix = new RotationScaleMatrix();
         RotationScaleMatrix actualMatrix = new RotationScaleMatrix();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(3453L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         transform.get(denseMatrix);
         CommonOps.invert(denseMatrix);
         transform.invert();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(denseMatrix.get(row, column), transform.getElement(row, column), EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform expectedTransform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform actualTransform = new RigidBodyTransform();
         actualTransform.setAndInvert(expectedTransform);
         expectedTransform.invert();
         GeometryBasicsTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(465416L);

      // Test against invert
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform inverse = new RigidBodyTransform(transform);
         inverse.invert();

         transform.multiply(inverse);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               if (row == column)
                  assertEquals(transform.getElement(row, column), 1.0, EPS);
               else
                  assertEquals(transform.getElement(row, column), 0.0, EPS);
            }
         }
      }

      // Test against EJML
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform t1 = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform t2 = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform t3 = new RigidBodyTransform(t1);
         t3.multiply(t2);

         DenseMatrix64F m1 = new DenseMatrix64F(4, 4);
         DenseMatrix64F m2 = new DenseMatrix64F(4, 4);
         DenseMatrix64F m3 = new DenseMatrix64F(4, 4);
         t1.get(m1);
         t2.get(m2);
         CommonOps.mult(m1, m2, m3);

         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(m3.get(row, column), t3.getElement(row, column), EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(465416L);

      // Test against invert
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform inverse = new RigidBodyTransform(transform);
         inverse.invert();

         transform.preMultiply(inverse);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               if (row == column)
                  assertEquals(transform.getElement(row, column), 1.0, EPS);
               else
                  assertEquals(transform.getElement(row, column), 0.0, EPS);
            }
         }
      }

      // Test against EJML
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform t1 = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform t2 = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform t3 = new RigidBodyTransform(t2);
         t3.preMultiply(t1);

         DenseMatrix64F m1 = new DenseMatrix64F(4, 4);
         DenseMatrix64F m2 = new DenseMatrix64F(4, 4);
         DenseMatrix64F m3 = new DenseMatrix64F(4, 4);
         t1.get(m1);
         t2.get(m2);
         CommonOps.mult(m1, m2, m3);

         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(m3.get(row, column), t3.getElement(row, column), EPS);
      }
   }

   @Test
   public void testTransformWithTuple() throws Exception
   {
      Random random = new Random(432L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      transform.get(matrix);

      { // Test transform(PointBasics pointToTransform)
         DenseMatrix64F ejmlPoint = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedPoint = new DenseMatrix64F(4, 1);

         Point3D point = GeometryBasicsRandomTools.generateRandomPoint3D(random);
         point.get(ejmlPoint);
         ejmlPoint.set(3, 0, 1.0);

         transform.transform(point);
         CommonOps.mult(matrix, ejmlPoint, ejmlTransformedPoint);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedPoint.get(i, 0), point.get(i), EPS);
      }

      { // Test transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         DenseMatrix64F ejmlPoint = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedPoint = new DenseMatrix64F(4, 1);

         Point3D point = GeometryBasicsRandomTools.generateRandomPoint3D(random);
         Point3D pointTransformed = new Point3D();
         point.get(ejmlPoint);
         ejmlPoint.set(3, 0, 1.0);

         transform.transform(point, pointTransformed);
         CommonOps.mult(matrix, ejmlPoint, ejmlTransformedPoint);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedPoint.get(i, 0), pointTransformed.get(i), EPS);
      }

      { // Test transform(VectorBasics vectorToTransform)
         DenseMatrix64F ejmlVector = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedVector = new DenseMatrix64F(4, 1);

         Vector3D vector = GeometryBasicsRandomTools.generateRandomVector3D(random);
         vector.get(ejmlVector);

         transform.transform(vector);
         CommonOps.mult(matrix, ejmlVector, ejmlTransformedVector);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedVector.get(i, 0), vector.get(i), EPS);
      }

      { // Test transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         DenseMatrix64F ejmlVector = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedVector = new DenseMatrix64F(4, 1);

         Vector3D vector = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Vector3D vectorTransformed = new Vector3D();
         vector.get(ejmlVector);

         transform.transform(vector, vectorTransformed);
         CommonOps.mult(matrix, ejmlVector, ejmlTransformedVector);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedVector.get(i, 0), vectorTransformed.get(i), EPS);
      }
   }

   @Test
   public void testTransformWithQuaternion() throws Exception
   {
      Random random = new Random(34534L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      Quaternion quaternionOriginal = GeometryBasicsRandomTools.generateRandomQuaternion(random);
      Quaternion quaternionExpected = new Quaternion();
      Quaternion quaternionActual = new Quaternion();

      transform.getRotation(quaternionExpected);
      quaternionExpected.multiply(quaternionOriginal);

      transform.transform(quaternionOriginal, quaternionActual);
      GeometryBasicsTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPS);

      quaternionActual.set(quaternionOriginal);
      transform.transform(quaternionActual);
      GeometryBasicsTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPS);
   }

   @Test
   public void testTransformWithVector4D() throws Exception
   {
      Random random = new Random(5634L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      Vector4D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vectorExpected = new Vector4D();
      Vector4D vectorActual = new Vector4D();

      Vector3D vector3D = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), vectorOriginal.getZ());
      transform.transform(vector3D);
      vectorExpected.set(vector3D);
      vectorExpected.setS(vectorOriginal.getS());

      transform.transform(vectorOriginal, vectorActual);
      GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      
      vectorActual.set(vectorOriginal);
      transform.transform(vectorActual);
      GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testTransformWithTuple2D() throws Exception
   {
      Random random = new Random(4353L);
      RigidBodyTransform transfom2D = new RigidBodyTransform();
      transfom2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(GeometryBasicsRandomTools.generateRandomVector3D(random));

      { // Test transform(Point2DBasics pointToTransform)
         Point2D pointOriginal = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         pointActual.set(pointOriginal);
         transfom2D.transform(pointActual);
         GeometryBasicsTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
         Point2D pointOriginal = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         pointActual.set(pointOriginal);
         transfom2D.transform(pointActual, true);
         GeometryBasicsTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed)
         Point2D pointOriginal = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         transfom2D.transform(pointOriginal, pointActual);
         GeometryBasicsTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
         Point2D pointOriginal = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         transfom2D.transform(pointOriginal, pointActual, true);
         GeometryBasicsTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D vectorExpected = new Vector2D();
         Vector2D vectorActual = new Vector2D();

         Vector3D vector = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), 0.0);
         transfom2D.transform(vector);
         vectorExpected.set(vector.getX(), vector.getY());

         vectorActual.set(vectorOriginal);
         transfom2D.transform(vectorActual);
         GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D vectorExpected = new Vector2D();
         Vector2D vectorActual = new Vector2D();

         Vector3D vector = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), 0.0);
         transfom2D.transform(vector);
         vectorExpected.set(vector.getX(), vector.getY());

         transfom2D.transform(vectorOriginal, vectorActual);
         GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);
      }
   }

   @Test
   public void testTransformWithMatrix3D() throws Exception
   {
      Random random = new Random(4534L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      Matrix3D matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      {
         Matrix3D m = new Matrix3D();
         transform.getRotation(m);
         matrixExpected.set(matrixOriginal);
         matrixExpected.preMultiply(m);
         matrixExpected.multiplyTransposeOther(m);
      }

      transform.transform(matrixOriginal, matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.set(matrixOriginal);
      transform.transform(matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testTransformWithRotationMatrix() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RotationMatrix matrixOriginal = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();
      
      transform.getRotation(matrixExpected);
      matrixExpected.multiply(matrixOriginal);

      matrixActual.set(matrixOriginal);
      transform.transform(matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testInverseTransformWithTuple() throws Exception
   {
      Random random = new Random(3454L);
      RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);

      { // Test inverseTransform(PointBasics pointToTransform)
         Point3D pointExpected = GeometryBasicsRandomTools.generateRandomPoint3D(random);
         Point3D pointActual = new Point3D();
         pointActual.set(pointExpected);
         transform.transform(pointActual);
         transform.inverseTransform(pointActual);
         GeometryBasicsTestTools.assertTuple3DEquals(pointExpected, pointActual, EPS);
      }

      { // Test inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point3D pointExpected = GeometryBasicsRandomTools.generateRandomPoint3D(random);
         Point3D pointActual = new Point3D();

         transform.inverseTransform(pointExpected, pointActual);
         transform.transform(pointActual);
         GeometryBasicsTestTools.assertTuple3DEquals(pointExpected, pointActual, EPS);
      }

      { // Test inverseTransform(VectorBasics vectorToTransform)
         Vector3D vectorExpected = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Vector3D vectorActual = new Vector3D();
         vectorActual.set(vectorExpected);
         transform.transform(vectorActual);
         transform.inverseTransform(vectorActual);
         GeometryBasicsTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector3D vectorExpected = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Vector3D vectorActual = new Vector3D();

         transform.inverseTransform(vectorExpected, vectorActual);
         transform.transform(vectorActual);
         GeometryBasicsTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPS);
      }
   }

   @Test
   public void testInverseTransformWithTuple2D() throws Exception
   {
      Random random = new Random(3454L);
      RigidBodyTransform transfom2D = new RigidBodyTransform();
      transfom2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(GeometryBasicsRandomTools.generateRandomVector3D(random));

      { // Test inverseTransform(Point2DBasics pointToTransform)
         Point2D pointExpected = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D pointActual = new Point2D();
         pointActual.set(pointExpected);
         transfom2D.transform(pointActual);
         transfom2D.inverseTransform(pointActual);
         GeometryBasicsTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D pointExpected = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D pointActual = new Point2D();

         transfom2D.inverseTransform(pointExpected, pointActual);
         transfom2D.transform(pointActual);
         GeometryBasicsTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test inverseTransform(VectorBasics vectorToTransform)
         Vector2D vectorExpected = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D vectorActual = new Vector2D();
         vectorActual.set(vectorExpected);
         transfom2D.transform(vectorActual);
         transfom2D.inverseTransform(vectorActual);
         GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector2D vectorExpected = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D vectorActual = new Vector2D();

         transfom2D.inverseTransform(vectorExpected, vectorActual);
         transfom2D.transform(vectorActual);
         GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);
      }
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(5464L);
      RigidBodyTransform transform = new RigidBodyTransform();
      double coeff;

      transform.setUnsafe(coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM00() == coeff);
      assertTrue(transform.getElement(0, 0) == coeff);
      transform.setUnsafe(0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM01() == coeff);
      assertTrue(transform.getElement(0, 1) == coeff);
      transform.setUnsafe(0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM02() == coeff);
      assertTrue(transform.getElement(0, 2) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM03() == coeff);
      assertTrue(transform.getElement(0, 3) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM10() == coeff);
      assertTrue(transform.getElement(1, 0) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM11() == coeff);
      assertTrue(transform.getElement(1, 1) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM12() == coeff);
      assertTrue(transform.getElement(1, 2) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM13() == coeff);
      assertTrue(transform.getElement(1, 3) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0);
      assertTrue(transform.getM20() == coeff);
      assertTrue(transform.getElement(2, 0) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0);
      assertTrue(transform.getM21() == coeff);
      assertTrue(transform.getElement(2, 1) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0);
      assertTrue(transform.getM22() == coeff);
      assertTrue(transform.getElement(2, 2) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble());
      assertTrue(transform.getM23() == coeff);
      assertTrue(transform.getElement(2, 3) == coeff);

      try
      {
         transform.getElement(0, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(1, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(2, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(3, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(4, 0);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      RigidBodyTransform t1 = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform t2 = new RigidBodyTransform();

      assertFalse(t1.equals(t2));
      assertFalse(t1.equals(null));
      assertFalse(t1.equals(new double[4]));
      t2.set(t1);
      assertTrue(t1.equals(t2));
      assertTrue(t1.equals((Object) t2));

      double smallestEpsilon = 1.0e-16;
      double[] coeffs = new double[16];

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            t2.set(t1);
            assertTrue(t1.equals(t2));
            t1.get(coeffs);
            coeffs[3 * row + column] += smallestEpsilon;
            t2.set(coeffs);
            assertFalse(t1.equals(t2));

            t2.set(t1);
            assertTrue(t1.equals(t2));
            t1.get(coeffs);
            coeffs[3 * row + column] -= smallestEpsilon;
            t2.set(coeffs);
            assertFalse(t1.equals(t2));
         }
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(2354L);
      RigidBodyTransform t1 = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform t2 = new RigidBodyTransform();
      double epsilon = 1.0e-3;
      double[] coeffs = new double[16];

      assertFalse(t1.epsilonEquals(t2, epsilon));
      t2.set(t1);
      assertTrue(t1.epsilonEquals(t2, epsilon));

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] += 0.999 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertTrue(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] += 1.001 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertFalse(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] -= 0.999 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertTrue(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] -= 1.001 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertFalse(t1.epsilonEquals(t2, epsilon));
         }
      }
   }
}
