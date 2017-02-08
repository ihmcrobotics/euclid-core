package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Tuple3DTools;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public class Vector4DTest
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(3453L);

      { // Test empty constructor
         Vector4D vector = new Vector4D();
         assertTrue(vector.getX() == 0.0);
         assertTrue(vector.getY() == 0.0);
         assertTrue(vector.getZ() == 0.0);
         assertTrue(vector.getS() == 0.0);
      }

      { // Test Vector4D(double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         Vector4D vector = new Vector4D(x, y, z, s);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test Vector4D(double[] vectorArray)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         double[] vectorArray = {x, y, z, s};
         Vector4D vector = new Vector4D(vectorArray);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test Vector4D(QuaternionReadOnly quaternion)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector4D vector = new Vector4D(quaternion);
         GeometryBasicsTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D(Tuple4DReadOnly other)
         Tuple4DReadOnly quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector4D vector = new Vector4D(quaternion);
         GeometryBasicsTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(45654L);

      { // Test set(Vector4D other)
         Vector4D vectorExpected = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4D vectorActual = new Vector4D();
         vectorActual.set(vectorExpected);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test set(double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         Vector4D vector = new Vector4D();
         vector.set(x, y, z, s);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test set(double[] vectorArray)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         double[] vectorArray = {x, y, z, s};
         Vector4D vector = new Vector4D();
         vector.set(vectorArray);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test set(double[] vectorArray, int startIndex)
         int startIndex = random.nextInt(10);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         double[] vectorArray = new double[4 + startIndex];
         vectorArray[startIndex + 0] = x;
         vectorArray[startIndex + 1] = y;
         vectorArray[startIndex + 2] = z;
         vectorArray[startIndex + 3] = s;
         Vector4D vector = new Vector4D();
         vector.set(vectorArray, startIndex);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test set(Tuple4DReadOnly other)
         Tuple4DReadOnly vectorExpected = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4D vectorActual = new Vector4D();
         vectorActual.set(vectorExpected);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test set(TupleReadOnly other)
         Vector3D vectorExpected = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Vector4D vectorActual = new Vector4D();
         vectorActual.set(vectorExpected);
         assertTrue(vectorActual.getX() == vectorExpected.getX());
         assertTrue(vectorActual.getY() == vectorExpected.getY());
         assertTrue(vectorActual.getZ() == vectorExpected.getZ());
         assertTrue(vectorActual.getS() == 0.0);
      }

      { // Test set(DenseMatrix64F matrix)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         DenseMatrix64F vectorDenseMatrix = new DenseMatrix64F(4, 1);
         vectorDenseMatrix.set(4, 1, true, x, y, z, s);
         Vector4D vector = new Vector4D();
         vector.set(vectorDenseMatrix);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test set(DenseMatrix64F matrix, int startRow)
         int startRow = random.nextInt(10);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         DenseMatrix64F vectorDenseMatrix = new DenseMatrix64F(4 + startRow, 1);
         vectorDenseMatrix.set(startRow + 0, 0, x);
         vectorDenseMatrix.set(startRow + 1, 0, y);
         vectorDenseMatrix.set(startRow + 2, 0, z);
         vectorDenseMatrix.set(startRow + 3, 0, s);
         Vector4D vector = new Vector4D();
         vector.set(vectorDenseMatrix, startRow);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }
   }

   @Test
   public void testGetSetWithIndex() throws Exception
   {
      Random random = new Random(4353L);
      Vector4D vector = new Vector4D();
      double coeff;

      vector.set(0, coeff = random.nextDouble());
      assertTrue(vector.get(0) == coeff);
      assertTrue(vector.getX() == coeff);
      vector.set(1, coeff = random.nextDouble());
      assertTrue(vector.get(1) == coeff);
      assertTrue(vector.getY() == coeff);
      vector.set(2, coeff = random.nextDouble());
      assertTrue(vector.get(2) == coeff);
      assertTrue(vector.getZ() == coeff);
      vector.set(3, coeff = random.nextDouble());
      assertTrue(vector.get(3) == coeff);
      assertTrue(vector.getS() == coeff);

      try
      {
         vector.set(4, 0.0);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // good
      }

      try
      {
         vector.get(4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // good
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Vector4D vector = new Vector4D();
      vector.setToNaN();
      for (int i = 0; i < 4; i++)
         assertTrue(Double.isNaN(vector.get(i)));
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(435L);
      Vector4D vector = GeometryBasicsRandomTools.generateRandomVector4D(random);
      vector.setToZero();
      for (int i = 0; i < 4; i++)
         assertTrue(vector.get(i) == 0.0);
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(435L);
      Vector4D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vectorNegated = new Vector4D(vectorOriginal);
      vectorNegated.negate();

      for (int i = 0; i < 4; i++)
         assertTrue(vectorOriginal.get(i) == -vectorNegated.get(i));
   }

   @Test
   public void testAbsolute() throws Exception
   {
      Random random = new Random(435L);
      Vector4D vectorOriginal = new Vector4D();
      vectorOriginal.set(-random.nextDouble(), -random.nextDouble(), -random.nextDouble(), -random.nextDouble());
      Vector4D vectorAbsolute = new Vector4D(vectorOriginal);
      vectorAbsolute.absolute();

      for (int i = 0; i < 4; i++)
         assertTrue(vectorOriginal.get(i) == -vectorAbsolute.get(i));

      vectorOriginal.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      vectorAbsolute.set(vectorOriginal);
      vectorAbsolute.absolute();

      for (int i = 0; i < 4; i++)
         assertTrue(vectorOriginal.get(i) == vectorAbsolute.get(i));
   }

   @Test
   public void testNormalize() throws Exception
   {
      Random random = new Random(435L);
      Vector4D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vector = new Vector4D(vectorOriginal);
      assertNotEquals(1.0, vector.length(), EPS);
      vector.normalize();
      assertEquals(1.0, vector.length(), EPS);
      assertTrue(vectorOriginal.dot(vector) > 0.0);

      vector.setToNaN();
      vector.normalize();
      for (int i = 0; i < 4; i++)
         assertTrue(Double.isNaN(vector.get(i)));
   }

   @Test
   public void testDot() throws Exception
   {
      Random random = new Random(32453L);
      Vector4D v1 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v2 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      assertEquals(Tuple4DTools.dot(v1, v2), v1.dot(v2), EPS);
   }

   @Test
   public void testLengthSquared() throws Exception
   {
      Random random = new Random(4353L);
      Vector4D vector = GeometryBasicsRandomTools.generateRandomVector4D(random);
      assertEquals(vector.dot(vector), vector.lengthSquared(), EPS);
   }

   @Test
   public void testLength() throws Exception
   {
      Random random = new Random(4353L);
      Vector4D vector = GeometryBasicsRandomTools.generateRandomVector4D(random);
      assertEquals(Math.sqrt(vector.lengthSquared()), vector.length(), EPS);
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(4353L);
      Vector4D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vectorScaled = new Vector4D();

      {
         double scale = random.nextDouble();
         vectorScaled.set(vectorOriginal);
         vectorScaled.scale(scale);
         for (int i = 0; i < 4; i++)
            assertEquals(scale * vectorOriginal.get(i), vectorScaled.get(i), EPS);
      }

      {
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();
         double scaleZ = random.nextDouble();
         double scaleS = random.nextDouble();
         double[] scale = {scaleX, scaleY, scaleZ, scaleS};
         vectorScaled.set(vectorOriginal);
         vectorScaled.scale(scaleX, scaleY, scaleZ, scaleS);
         for (int i = 0; i < 4; i++)
            assertEquals(scale[i] * vectorOriginal.get(i), vectorScaled.get(i), EPS);
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(34543L);
      Vector4D v0 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vf = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vector = new Vector4D();
      double alpha = random.nextDouble();

      vector.interpolate(v0, vf, alpha);
      for (int i = 0; i < 4; i++)
         assertEquals(Tuple3DTools.interpolate(v0.get(i), vf.get(i), alpha), vector.get(i), EPS);

      vector.set(v0);
      vector.interpolate(vf, alpha);
      for (int i = 0; i < 4; i++)
         assertEquals(Tuple3DTools.interpolate(v0.get(i), vf.get(i), alpha), vector.get(i), EPS);
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(43543L);
      Vector4D v1 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v2 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v3 = GeometryBasicsRandomTools.generateRandomVector4D(random);

      v3.set(v1);
      v3.add(v2);

      for (int i = 0; i < 4; i++)
         assertEquals(v1.get(i) + v2.get(i), v3.get(i), EPS);
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(43543L);
      Vector4D v1 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v2 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v3 = GeometryBasicsRandomTools.generateRandomVector4D(random);

      v3.set(v1);
      v3.sub(v2);

      for (int i = 0; i < 4; i++)
         assertEquals(v1.get(i) - v2.get(i), v3.get(i), EPS);
   }

   @Test
   public void testIndividualSetters() throws Exception
   {
      Random random = new Random(3453L);
      Vector4D vector = new Vector4D();
      double coeff;

      vector.setX(coeff = random.nextDouble());
      assertTrue(coeff == vector.getX());
      vector.setY(coeff = random.nextDouble());
      assertTrue(coeff == vector.getY());
      vector.setZ(coeff = random.nextDouble());
      assertTrue(coeff == vector.getZ());
      vector.setS(coeff = random.nextDouble());
      assertTrue(coeff == vector.getS());
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(234534L);
      Vector4D vector = GeometryBasicsRandomTools.generateRandomVector4D(random);

      { // Test get(double[] tupleArray)
         double[] tupleArray = new double[4];
         vector.get(tupleArray);

         for (int i = 0; i < 4; i++)
            assertTrue(vector.get(i) == tupleArray[i]);
      }

      { // Test get(double[] tupleArray, int startIndex)
         int startIndex = random.nextInt(10);
         double[] tupleArray = new double[4 + startIndex];
         vector.get(tupleArray, startIndex);

         for (int i = 0; i < 4; i++)
            assertTrue(vector.get(i) == tupleArray[i + startIndex]);
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F tupleMatrixToPack = new DenseMatrix64F(4, 1);
         vector.get(tupleMatrixToPack);

         for (int i = 0; i < 4; i++)
            assertTrue(vector.get(i) == tupleMatrixToPack.get(i, 0));
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack, int startRow)
         int startRow = random.nextInt(10);
         DenseMatrix64F tupleMatrixToPack = new DenseMatrix64F(4 + startRow, 1);
         vector.get(tupleMatrixToPack, startRow);

         for (int i = 0; i < 4; i++)
            assertTrue(vector.get(i) == tupleMatrixToPack.get(i + startRow, 0));
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack, int startRow, int startColumn)
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DenseMatrix64F tupleMatrixToPack = new DenseMatrix64F(4 + startRow, 1 + startColumn);
         vector.get(tupleMatrixToPack, startRow, startColumn);

         for (int i = 0; i < 4; i++)
            assertTrue(vector.get(i) == tupleMatrixToPack.get(i + startRow, startColumn));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Vector4D vector = new Vector4D();

      Assert.assertFalse(vector.containsNaN());

      vector.set(Double.NaN, 0.0, 0.0, 0.0);
      Assert.assertTrue(vector.containsNaN());

      vector.set(0.0, Double.NaN, 0.0, 0.0);
      Assert.assertTrue(vector.containsNaN());

      vector.set(0.0, 0.0, Double.NaN, 0.0);
      Assert.assertTrue(vector.containsNaN());

      vector.set(0.0, 0.0, 0.0, Double.NaN);
      Assert.assertTrue(vector.containsNaN());
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      Vector4D v1 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v2 = new Vector4D();

      assertFalse(v1.equals(v2));
      assertFalse(v1.equals(null));
      assertFalse(v1.equals(new double[4]));
      v2.set(v1);
      assertTrue(v1.equals(v2));
      assertTrue(v1.equals((Object) v2));

      double smallestEpsilon = 1.0e-16;

      for (int row = 0; row < 4; row++)
      {
         v2.set(v1);
         assertTrue(v1.equals(v2));
         v2.set(row, v2.get(row) + smallestEpsilon);
         assertFalse(v1.equals(v2));

         v2.set(v1);
         assertTrue(v1.equals(v2));
         v2.set(row, v2.get(row) - smallestEpsilon);
         assertFalse(v1.equals(v2));
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(2354L);
      Vector4D v1 = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D v2 = new Vector4D();
      double epsilon = 1.0e-3;

      assertFalse(v1.epsilonEquals(v2, epsilon));
      v2.set(v1);
      assertTrue(v1.epsilonEquals(v2, epsilon));

      for (int row = 0; row < 4; row++)
      {
         v2.set(v1);
         assertTrue(v1.epsilonEquals(v2, epsilon));
         v2.set(row, v2.get(row) + 0.999 * epsilon);
         assertTrue(v1.epsilonEquals(v2, epsilon));

         v2.set(v1);
         assertTrue(v1.epsilonEquals(v2, epsilon));
         v2.set(row, v2.get(row) + 1.001 * epsilon);
         assertFalse(v1.epsilonEquals(v2, epsilon));

         v2.set(v1);
         assertTrue(v1.epsilonEquals(v2, epsilon));
         v2.set(row, v2.get(row) - 0.999 * epsilon);
         assertTrue(v1.epsilonEquals(v2, epsilon));

         v2.set(v1);
         assertTrue(v1.epsilonEquals(v2, epsilon));
         v2.set(row, v2.get(row) - 1.001 * epsilon);
         assertFalse(v1.epsilonEquals(v2, epsilon));
      }
   }
}
