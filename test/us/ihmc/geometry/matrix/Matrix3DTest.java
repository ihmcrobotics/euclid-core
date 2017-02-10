package us.ihmc.geometry.matrix;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.exceptions.SingularMatrixException;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple3D.Vector3D;

public class Matrix3DTest extends Matrix3DBasicsTest<Matrix3D>
{
   public static final int NUMBER_OF_ITERATIONS = 100;
   public static final double EPS = 1.0e-10;

   @Test
   public void testMatrix3D()
   {
      Random random = new Random(65431L);
      Matrix3D matrix = new Matrix3D(), matrixCopy;

      for (int k = 0; k < NUMBER_OF_ITERATIONS; k++)
      { // Test Matrix3D(double[] matrixArray)
         double[] matrixArray = new double[9];
         double[] matrixArrayCopy = new double[9];

         for (int i = 0; i < matrixArray.length; i++)
            matrixArray[i] = matrixArrayCopy[i] = random.nextDouble();

         matrix = matrixCopy = new Matrix3D(matrixArray);
         Matrix3D expectedMatrix = new Matrix3D();
         expectedMatrix.set(matrixArray);

         for (int j = 0; j < matrixArray.length; j++)
            Assert.assertTrue(matrixArray[j] == matrixArrayCopy[j]);

         Assert.assertTrue(matrix.getM00() == matrixArray[0]);
         Assert.assertTrue(matrix.getM01() == matrixArray[1]);
         Assert.assertTrue(matrix.getM02() == matrixArray[2]);
         Assert.assertTrue(matrix.getM10() == matrixArray[3]);
         Assert.assertTrue(matrix.getM11() == matrixArray[4]);
         Assert.assertTrue(matrix.getM12() == matrixArray[5]);
         Assert.assertTrue(matrix.getM20() == matrixArray[6]);
         Assert.assertTrue(matrix.getM21() == matrixArray[7]);
         Assert.assertTrue(matrix.getM22() == matrixArray[8]);

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, expectedMatrix, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Matrix3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         Matrix3D expectedMatrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         matrix = new Matrix3D(expectedMatrix.getM00(), expectedMatrix.getM01(), expectedMatrix.getM02(), expectedMatrix.getM10(), expectedMatrix.getM11(),
                               expectedMatrix.getM12(), expectedMatrix.getM20(), expectedMatrix.getM21(), expectedMatrix.getM22());

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, expectedMatrix, EPS);
      }

      for (int k = 0; k < NUMBER_OF_ITERATIONS; k++)
      { // Test Matrix3D(Matrix3DBasics other)
         double[] matrixArray = new double[9];

         for (int i = 0; i < matrixArray.length; i++)
            matrixArray[i] = random.nextDouble();

         matrix = new Matrix3D(matrixArray);
         Matrix3D matrix2 = new Matrix3D(matrix);

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrix2, EPS);

         Assert.assertTrue(matrix.getM00() == matrixArray[0]);
         Assert.assertTrue(matrix.getM01() == matrixArray[1]);
         Assert.assertTrue(matrix.getM02() == matrixArray[2]);
         Assert.assertTrue(matrix.getM10() == matrixArray[3]);
         Assert.assertTrue(matrix.getM11() == matrixArray[4]);
         Assert.assertTrue(matrix.getM12() == matrixArray[5]);
         Assert.assertTrue(matrix.getM20() == matrixArray[6]);
         Assert.assertTrue(matrix.getM21() == matrixArray[7]);
         Assert.assertTrue(matrix.getM22() == matrixArray[8]);
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(564L);
      Matrix3D matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertFalse(matrix.getElement(row, column) == 0.0);
         }
      }

      matrix.setToZero();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(6345L);
      Matrix3D actualMatrix = new Matrix3D();

      for (int j = 0; j < NUMBER_OF_ITERATIONS; j++)
      {
         double m00 = random.nextDouble();
         double m01 = random.nextDouble();
         double m02 = random.nextDouble();
         double m10 = random.nextDouble();
         double m11 = random.nextDouble();
         double m12 = random.nextDouble();
         double m20 = random.nextDouble();
         double m21 = random.nextDouble();
         double m22 = random.nextDouble();
         actualMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         assertTrue(m00 == actualMatrix.getM00());
         assertTrue(m01 == actualMatrix.getM01());
         assertTrue(m02 == actualMatrix.getM02());
         assertTrue(m10 == actualMatrix.getM10());
         assertTrue(m11 == actualMatrix.getM11());
         assertTrue(m12 == actualMatrix.getM12());
         assertTrue(m20 == actualMatrix.getM20());
         assertTrue(m21 == actualMatrix.getM21());
         assertTrue(m22 == actualMatrix.getM22());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Matrix3D expectedMatrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         actualMatrix.set(expectedMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Matrix3D expectedMatrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         actualMatrix.set((Matrix3DReadOnly<?>) expectedMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double[] matrixArray = new double[9];
         for (int index = 0; index < 9; index++)
            matrixArray[index] = random.nextDouble();
         actualMatrix.set(matrixArray);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(actualMatrix.getElement(row, column) == matrixArray[row * 3 + column]);
            }
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         DenseMatrix64F denseMatrix = RandomMatrices.createRandom(3, 3, random);
         actualMatrix.set(denseMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(actualMatrix.getElement(row, column) == denseMatrix.get(row, column));
            }
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DenseMatrix64F denseMatrix = RandomMatrices.createRandom(3 + startRow, 3 + startColumn, random);
         actualMatrix.set(startRow, startColumn, denseMatrix);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               assertTrue(actualMatrix.getElement(row, column) == denseMatrix.get(row + startRow, column + startColumn));
            }
         }
      }
   }

   @Test
   public void testSetToTildeForm()
   {
      Random random = new Random(646L);
      Matrix3D tildeMatrix = new Matrix3D();
      Vector3D vector, vectorCopy;
      Vector3D vector2, vector2Copy;
      Vector3D vector3 = new Vector3D();
      Vector3D expectedVector3 = new Vector3D();

      for (int k = 0; k < NUMBER_OF_ITERATIONS; k++)
      {
         vector = vectorCopy = GeometryBasicsRandomTools.generateRandomVector3D(random);
         vector2 = vector2Copy = GeometryBasicsRandomTools.generateRandomVector3D(random);
         vector3.cross(vector, vector2);

         tildeMatrix.set(vector.getX(), vector.getY(), vector.getZ(), vector2.getX(), vector2.getY(), vector2.getZ(), vector3.getX(), vector3.getY(),
                         vector3.getZ());
         tildeMatrix.setToTildeForm(vector);

         tildeMatrix.transform(vector2, expectedVector3);

         GeometryBasicsTestTools.assertRotationVectorEquals(vector, vectorCopy, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(vector2, vector2Copy, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(vector3, expectedVector3, EPS);
      }
   }

   @Test
   public void testAdd()
   {
      Random random = new Random(6345L);
      Matrix3D a = new Matrix3D();
      Matrix3D b = new Matrix3D();
      // c = a + b
      Matrix3D c = new Matrix3D();

      for (int p = 0; p < NUMBER_OF_ITERATIONS; p++)
      {
         c.setToNaN();

         a = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         b = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         c.set(a);
         c.add(b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij + bij, cij, EPS);
            }
         }

         c.setToNaN();
         a = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         b = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         c.add(a, b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij + bij, cij, EPS);
            }
         }
      }
   }

   @Test
   public void testSub()
   {
      Random random = new Random(6345L);
      Matrix3D a = new Matrix3D();
      Matrix3D b = new Matrix3D();
      // c = a - b
      Matrix3D c = new Matrix3D();

      for (int p = 0; p < NUMBER_OF_ITERATIONS; p++)
      {
         c.setToNaN();

         a = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         b = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         c.set(a);
         c.sub(b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij - bij, cij, EPS);
            }
         }

         c.setToNaN();
         a = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         b = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         c.sub(a, b);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double aij = a.getElement(row, column);
               double bij = b.getElement(row, column);
               double cij = c.getElement(row, column);
               assertEquals(aij - bij, cij, EPS);
            }
         }
      }
   }

   @Test
   public void testScale()
   {
      Random random = new Random(6345L);
      double scalar = Double.NaN;
      Matrix3D matrixOriginal = new Matrix3D();
      Matrix3D scaledMatrix = new Matrix3D();

      Vector3D scales = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         scalar = random.nextDouble();

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scale(scalar);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               assertEquals(scalar * mij, sij, EPS);
            }
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         scalar = random.nextDouble();
         int scaledRow = random.nextInt(3);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleRow(scaledRow, scalar);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               if (row == scaledRow)
                  assertEquals(scalar * mij, sij, EPS);
               else
                  assertEquals(mij, sij, EPS);
            }
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         scalar = random.nextDouble();
         int scaledColumn = random.nextInt(3);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleColumn(scaledColumn, scalar);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               if (column == scaledColumn)
                  assertEquals(scalar * mij, sij, EPS);
               else
                  assertEquals(mij, sij, EPS);
            }
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         scales = GeometryBasicsRandomTools.generateRandomRotationVector(random, 2.0);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleColumns(scales.getX(), scales.getY(), scales.getZ());

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               assertEquals(scales.get(column) * mij, sij, EPS);
            }
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         scales = GeometryBasicsRandomTools.generateRandomRotationVector(random, 2.0);

         scaledMatrix.set(matrixOriginal);
         scaledMatrix.scaleRows(scales.getX(), scales.getY(), scales.getZ());

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               double mij = matrixOriginal.getElement(row, column);
               double sij = scaledMatrix.getElement(row, column);
               assertEquals(scales.get(row) * mij, sij, EPS);
            }
         }
      }

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM00(scalar);
      assertEquals(scalar * matrixOriginal.getM00(), scaledMatrix.getM00(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM01(scalar);
      assertEquals(scalar * matrixOriginal.getM01(), scaledMatrix.getM01(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM02(scalar);
      assertEquals(scalar * matrixOriginal.getM02(), scaledMatrix.getM02(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM10(scalar);
      assertEquals(scalar * matrixOriginal.getM10(), scaledMatrix.getM10(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM11(scalar);
      assertEquals(scalar * matrixOriginal.getM11(), scaledMatrix.getM11(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM12(scalar);
      assertEquals(scalar * matrixOriginal.getM12(), scaledMatrix.getM12(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM20(scalar);
      assertEquals(scalar * matrixOriginal.getM20(), scaledMatrix.getM20(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM21(scalar);
      assertEquals(scalar * matrixOriginal.getM21(), scaledMatrix.getM21(), EPS);

      matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      scaledMatrix.set(matrixOriginal);
      scalar = random.nextDouble();
      scaledMatrix.scaleM22(scalar);
      assertEquals(scalar * matrixOriginal.getM22(), scaledMatrix.getM22(), EPS);

      // Test some exceptions
      try
      {
         scaledMatrix.scaleRow(3, scalar);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         scaledMatrix.scaleColumn(3, scalar);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testMultiplyOuter()
   {
      Random random = new Random(51665L);
      Matrix3D matrix, expectedMatrix = new Matrix3D();
      Matrix3D matrix2, matrix2Copy = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyOuter()
         matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         expectedMatrix.set(matrix);

         matrix.multiplyOuter();
         Matrix3DTools.multiplyTransposeRight(expectedMatrix, expectedMatrix, expectedMatrix);

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, expectedMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyOuter(Matrix3DBasics other)
         matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         expectedMatrix.set(matrix);
         matrix2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         matrix2Copy.set(matrix2);

         matrix.setAndMultiplyOuter(matrix2);
         expectedMatrix.set(matrix2);
         expectedMatrix.multiplyOuter();

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, expectedMatrix, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrix2, matrix2Copy, EPS);
      }
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(54654L);
      Matrix3D matrix = new Matrix3D();
      Matrix3D expectedInvert = new Matrix3D();
      Matrix3D actualInvert = new Matrix3D();

      actualInvert.setToZero();
      try
      {
         actualInvert.invert();
         fail("Should have thrown an exception");
      }
      catch (SingularMatrixException e)
      {
         // Good
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3DTools.invert(matrix, expectedInvert);
         actualInvert.set(matrix);
         actualInvert.invert();
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedInvert, actualInvert, EPS);

         matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3DTools.invert(matrix, expectedInvert);
         actualInvert.setAndInvert(matrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedInvert, actualInvert, EPS);
      }
   }

   @Test
   public void testNormalize() throws Exception
   {
      Random random = new Random(541654L);
      Matrix3D actualMatrix = new Matrix3D();
      Matrix3D expectedMatrix = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Matrix3D matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random, 2.0);

         expectedMatrix.set(matrix);
         Matrix3DTools.normalize(expectedMatrix);
         actualMatrix.set(matrix);
         actualMatrix.normalize();
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Matrix3D matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random, 2.0);

         expectedMatrix.set(matrix);
         Matrix3DTools.normalize(expectedMatrix);
         actualMatrix.setAndNormalize(matrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
      }
   }

   @Test
   public void testTranspose() throws Exception
   {
      Random random = new Random(65451L);
      Matrix3D matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      Matrix3D matrixTranspose = new Matrix3D();

      matrixTranspose.set(matrix);
      matrixTranspose.transpose();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertEquals(matrix.getElement(row, column), matrixTranspose.getElement(column, row), EPS);
         }
      }

      matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      matrixTranspose.setAndTranspose(matrix);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertEquals(matrix.getElement(row, column), matrixTranspose.getElement(column, row), EPS);
         }
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiply(m1, m2, expected);
         actual.set(m1);
         actual.multiply(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyTransposeThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyTransposeLeft(m1, m2, expected);
         actual.set(m1);
         actual.multiplyTransposeThis(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyInvertLeft(m1, m2, expected);
         actual.set(m1);
         actual.multiplyInvertThis(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyTransposeOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyTransposeRight(m1, m2, expected);
         actual.set(m1);
         actual.multiplyTransposeOther(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyInvertRight(m1, m2, expected);
         actual.set(m1);
         actual.multiplyInvertOther(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);

         RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Matrix3DTools.multiplyInvertRight(m1, rotationMatrix, expected);
         actual.set(m1);
         actual.multiplyInvertOther(rotationMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);

         RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         Matrix3DTools.multiplyInvertRight(m1, rotationScaleMatrix, expected);
         actual.set(m1);
         actual.multiplyInvertOther(rotationScaleMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyTransposeBoth() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyTransposeBoth(m1, m2, expected);
         actual.set(m1);
         actual.multiplyTransposeBoth(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiply(m2, m1, expected);
         actual.set(m1);
         actual.preMultiply(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyTransposeThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyTransposeRight(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyTransposeThis(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyInvertRight(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertThis(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyTransposeOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyTransposeLeft(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyTransposeOther(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyInvertLeft(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertOther(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);

         RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Matrix3DTools.multiplyInvertLeft(rotationMatrix, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertOther(rotationMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);

         RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         Matrix3DTools.multiplyInvertLeft(rotationScaleMatrix, m1, expected);
         actual.set(m1);
         actual.preMultiplyInvertOther(rotationScaleMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyTransposeBoth() throws Exception
   {
      Random random = new Random(65561L);
      Matrix3D m1 = new Matrix3D();
      Matrix3D m2 = new Matrix3D();
      Matrix3D expected = new Matrix3D();
      Matrix3D actual = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         m2 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.multiplyTransposeBoth(m2, m1, expected);
         actual.set(m1);
         actual.preMultiplyTransposeBoth(m2);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testSetRow() throws Exception
   {
      Random random = new Random(6465L);
      Matrix3D matrix = new Matrix3D();

      double[] rowArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};
      matrix.setRow(0, rowArray);
      assertTrue(matrix.getM00() == rowArray[0]);
      assertTrue(matrix.getM01() == rowArray[1]);
      assertTrue(matrix.getM02() == rowArray[2]);

      matrix.setRow(1, rowArray);
      assertTrue(matrix.getM10() == rowArray[0]);
      assertTrue(matrix.getM11() == rowArray[1]);
      assertTrue(matrix.getM12() == rowArray[2]);

      matrix.setRow(2, rowArray);
      assertTrue(matrix.getM20() == rowArray[0]);
      assertTrue(matrix.getM21() == rowArray[1]);
      assertTrue(matrix.getM22() == rowArray[2]);

      try
      {
         matrix.setRow(3, rowArray);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      matrix.setRow(0, x, y, z);
      assertTrue(matrix.getM00() == x);
      assertTrue(matrix.getM01() == y);
      assertTrue(matrix.getM02() == z);

      matrix.setRow(1, x, y, z);
      assertTrue(matrix.getM10() == x);
      assertTrue(matrix.getM11() == y);
      assertTrue(matrix.getM12() == z);

      matrix.setRow(2, x, y, z);
      assertTrue(matrix.getM20() == x);
      assertTrue(matrix.getM21() == y);
      assertTrue(matrix.getM22() == z);

      try
      {
         matrix.setRow(3, x, y, z);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      Vector3D rowVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      matrix.setRow(0, rowVector);
      assertTrue(matrix.getM00() == rowVector.get(0));
      assertTrue(matrix.getM01() == rowVector.get(1));
      assertTrue(matrix.getM02() == rowVector.get(2));

      matrix.setRow(1, rowVector);
      assertTrue(matrix.getM10() == rowVector.get(0));
      assertTrue(matrix.getM11() == rowVector.get(1));
      assertTrue(matrix.getM12() == rowVector.get(2));

      matrix.setRow(2, rowVector);
      assertTrue(matrix.getM20() == rowVector.get(0));
      assertTrue(matrix.getM21() == rowVector.get(1));
      assertTrue(matrix.getM22() == rowVector.get(2));

      try
      {
         matrix.setRow(3, rowVector);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testSetColumn() throws Exception
   {
      Random random = new Random(6465L);
      Matrix3D matrix = new Matrix3D();

      double[] columnArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};
      matrix.setColumn(0, columnArray);
      assertTrue(matrix.getM00() == columnArray[0]);
      assertTrue(matrix.getM10() == columnArray[1]);
      assertTrue(matrix.getM20() == columnArray[2]);

      matrix.setColumn(1, columnArray);
      assertTrue(matrix.getM01() == columnArray[0]);
      assertTrue(matrix.getM11() == columnArray[1]);
      assertTrue(matrix.getM21() == columnArray[2]);

      matrix.setColumn(2, columnArray);
      assertTrue(matrix.getM02() == columnArray[0]);
      assertTrue(matrix.getM12() == columnArray[1]);
      assertTrue(matrix.getM22() == columnArray[2]);

      try
      {
         matrix.setColumn(3, columnArray);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      matrix.setColumn(0, x, y, z);
      assertTrue(matrix.getM00() == x);
      assertTrue(matrix.getM10() == y);
      assertTrue(matrix.getM20() == z);

      matrix.setColumn(1, x, y, z);
      assertTrue(matrix.getM01() == x);
      assertTrue(matrix.getM11() == y);
      assertTrue(matrix.getM21() == z);

      matrix.setColumn(2, x, y, z);
      assertTrue(matrix.getM02() == x);
      assertTrue(matrix.getM12() == y);
      assertTrue(matrix.getM22() == z);

      try
      {
         matrix.setColumn(3, x, y, z);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      Vector3D columnVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      matrix.setColumn(0, columnVector);
      assertTrue(matrix.getM00() == columnVector.get(0));
      assertTrue(matrix.getM10() == columnVector.get(1));
      assertTrue(matrix.getM20() == columnVector.get(2));

      matrix.setColumn(1, columnVector);
      assertTrue(matrix.getM01() == columnVector.get(0));
      assertTrue(matrix.getM11() == columnVector.get(1));
      assertTrue(matrix.getM21() == columnVector.get(2));

      matrix.setColumn(2, columnVector);
      assertTrue(matrix.getM02() == columnVector.get(0));
      assertTrue(matrix.getM12() == columnVector.get(1));
      assertTrue(matrix.getM22() == columnVector.get(2));

      try
      {
         matrix.setColumn(3, columnVector);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testSetGetElement() throws Exception
   {
      Random random = new Random(5464L);
      Matrix3D matrix = new Matrix3D();
      double coeff;

      matrix.setElement(0, 0, coeff = random.nextDouble());
      assertTrue(matrix.getM00() == coeff);
      assertTrue(matrix.getElement(0, 0) == coeff);
      matrix.setElement(0, 1, coeff = random.nextDouble());
      assertTrue(matrix.getM01() == coeff);
      assertTrue(matrix.getElement(0, 1) == coeff);
      matrix.setElement(0, 2, coeff = random.nextDouble());
      assertTrue(matrix.getM02() == coeff);
      assertTrue(matrix.getElement(0, 2) == coeff);
      matrix.setElement(1, 0, coeff = random.nextDouble());
      assertTrue(matrix.getM10() == coeff);
      assertTrue(matrix.getElement(1, 0) == coeff);
      matrix.setElement(1, 1, coeff = random.nextDouble());
      assertTrue(matrix.getM11() == coeff);
      assertTrue(matrix.getElement(1, 1) == coeff);
      matrix.setElement(1, 2, coeff = random.nextDouble());
      assertTrue(matrix.getM12() == coeff);
      assertTrue(matrix.getElement(1, 2) == coeff);
      matrix.setElement(2, 0, coeff = random.nextDouble());
      assertTrue(matrix.getM20() == coeff);
      assertTrue(matrix.getElement(2, 0) == coeff);
      matrix.setElement(2, 1, coeff = random.nextDouble());
      assertTrue(matrix.getM21() == coeff);
      assertTrue(matrix.getElement(2, 1) == coeff);
      matrix.setElement(2, 2, coeff = random.nextDouble());
      assertTrue(matrix.getM22() == coeff);
      assertTrue(matrix.getElement(2, 2) == coeff);

      try
      {
         matrix.setElement(0, 3, coeff);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         matrix.setElement(1, 3, coeff);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         matrix.setElement(2, 3, coeff);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         matrix.setElement(3, 0, coeff);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testFieldSettersAndGetters() throws Exception
   {
      Random random = new Random(654654L);
      Matrix3D matrix = new Matrix3D();
      double coeff;

      matrix.setM00(coeff = random.nextDouble());
      assertTrue(matrix.getM00() == coeff);
      matrix.setM01(coeff = random.nextDouble());
      assertTrue(matrix.getM01() == coeff);
      matrix.setM02(coeff = random.nextDouble());
      assertTrue(matrix.getM02() == coeff);
      matrix.setM10(coeff = random.nextDouble());
      assertTrue(matrix.getM10() == coeff);
      matrix.setM11(coeff = random.nextDouble());
      assertTrue(matrix.getM11() == coeff);
      matrix.setM12(coeff = random.nextDouble());
      assertTrue(matrix.getM12() == coeff);
      matrix.setM20(coeff = random.nextDouble());
      assertTrue(matrix.getM20() == coeff);
      matrix.setM21(coeff = random.nextDouble());
      assertTrue(matrix.getM21() == coeff);
      matrix.setM22(coeff = random.nextDouble());
      assertTrue(matrix.getM22() == coeff);
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      Matrix3D m1 = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      Matrix3D m2 = new Matrix3D();

      assertFalse(m1.equals(m2));
      assertFalse(m1.equals(null));
      assertFalse(m1.equals(new double[4]));
      m2.set(m1);
      assertTrue(m1.equals(m2));
      assertTrue(m1.equals((Object) m2));

      double smallestEpsilon = 1.0e-16;

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            m2.set(m1);
            assertTrue(m1.equals(m2));
            m2.setElement(row, column, m2.getElement(row, column) + smallestEpsilon);
            assertFalse(m1.equals(m2));

            m2.set(m1);
            assertTrue(m1.equals(m2));
            m2.setElement(row, column, m2.getElement(row, column) - smallestEpsilon);
            assertFalse(m1.equals(m2));
         }
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Matrix3D tuple1 = createRandomMatrix(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         int row = random.nextInt(3);
         int column = random.nextInt(3);
         tuple1.setElement(row, column, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         Matrix3D original = createRandomMatrix(random);
         Matrix3D expected = createEmptyMatrix();
         Matrix3D actual = createEmptyMatrix();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
         Matrix3D original = createRandomMatrix(random);
         Matrix3D expected = createEmptyMatrix();
         Matrix3D actual = createEmptyMatrix();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AffineTransform transform = GeometryBasicsRandomTools.generateRandomAffineTransform(random);
         Matrix3D original = createRandomMatrix(random);
         Matrix3D expected = createEmptyMatrix();
         Matrix3D actual = createEmptyMatrix();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Override
   public Matrix3D createEmptyMatrix()
   {
      return new Matrix3D();
   }

   @Override
   public Matrix3D createMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return new Matrix3D(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   @Override
   public Matrix3D createRandomMatrix(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomMatrix3D(random);
   }
}