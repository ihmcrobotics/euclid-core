package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class Quaternion32Test
{
   public static final int NUMBER_OF_ITERATIONS = QuaternionTest.NUMBER_OF_ITERATIONS;
   public static final float EPS = (float) 1e-6;

   @Test
   public void testQuaternion32()
   {
      Random random = new Random(613615L);
      Quaternion32 quaternion = new Quaternion32();
      Quaternion32 quaternionCopy;
      Quaternion32 expected = new Quaternion32();

      { // Test Quaternion32()
         expected.setToZero();
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(QuaternionBasics other)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         Quaternion32 quaternion2 = new Quaternion32((QuaternionReadOnly) quaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(float[] quaternionArray)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         float[] quaternionArray;
         quaternionArray = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32()};

         quaternion = new Quaternion32(quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(RotationMatrix rotationMatrix) about X-axis
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         quaternion = new Quaternion32(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, (QuaternionBasics) expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion32(VectorBasics rotationVector)
         Vector rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         quaternion = new Quaternion32(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion((VectorReadOnly) rotationVector, (QuaternionBasics) expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
      }
   }

   @Test
   @Ignore
   public void testQuaternion32Failing() // Fails because of normalizeAndLimitToPiMinusPi()
   {
      Random random = new Random(613615L);
      Quaternion quaternion = new Quaternion();
      Quaternion expected = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion = new Quaternion(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }
   }

   @Test
   public void testConjugate()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternionCopy.set(quaternion);

         { // Test conjugate()
            quaternion.conjugate();

            Assert.assertEquals(quaternion.getX(), -quaternionCopy.getX(), EPS);
            Assert.assertEquals(quaternion.getY(), -quaternionCopy.getY(), EPS);
            Assert.assertEquals(quaternion.getZ(), -quaternionCopy.getZ(), EPS);
            Assert.assertEquals(quaternion.getS(), quaternionCopy.getS(), EPS);
         }

         { // Test conjugate (QuaternionBasics other)
            Quaternion32 quaternion2 = new Quaternion32();
            quaternion2.setAndConjugate(quaternionCopy);

            Assert.assertTrue(quaternion2.getX() == -quaternionCopy.getX());
            Assert.assertTrue(quaternion2.getY() == -quaternionCopy.getY());
            Assert.assertTrue(quaternion2.getZ() == -quaternionCopy.getZ());
            Assert.assertTrue(quaternion2.getS() ==  quaternionCopy.getS());
         }
      }
   }

   @Test
   public void testNegate()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, expected;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         { // Test conjugate()
            quaternion = new Quaternion32(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32());

            quaternion.negate();

            Assert.assertEquals(expected.getX(), -quaternion.getX(), EPS);
            Assert.assertEquals(expected.getY(), -quaternion.getY(), EPS);
            Assert.assertEquals(expected.getZ(), -quaternion.getZ(), EPS);
            Assert.assertEquals(expected.getS(), -quaternion.getS(), EPS);
         }

         { // Test conjugate (QuaternionBasics other)
            quaternion = new Quaternion32(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32());

            Quaternion32 quaternion2 = new Quaternion32();
            quaternion2.setAndNegate(quaternion);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);

            Assert.assertTrue(quaternion2.getX() == -quaternion.getX());
            Assert.assertTrue(quaternion2.getY() == -quaternion.getY());
            Assert.assertTrue(quaternion2.getZ() == -quaternion.getZ());
            Assert.assertTrue(quaternion2.getS() == -quaternion.getS());
         }
      }
   }

   @Test
   public void testNormalize()
   {
      // TODO reimplement me
   }

   @Test
   public void testNormalizeAndLimitToPiMinusPi()
   {
      // TODO reimplement me
   }

   @Test
   public void testNormSquared()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         double normSquared = quaternion.normSquared();
         Assert.assertEquals(normSquared, 1.0, EPS);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternionCopy, quaternionCopy, EPS);
      }
   }

   @Test
   public void testSetToZero()
   {
      Quaternion32 quaternion = new Quaternion32();
      quaternion.setToZero();

      Assert.assertTrue(quaternion.getX() == 0.0);
      Assert.assertTrue(quaternion.getY() == 0.0);
      Assert.assertTrue(quaternion.getZ() == 0.0);
      Assert.assertTrue(quaternion.getS() == 1.0);
   }

   @Test
   public void testSetToNaN()
   {
      Quaternion32 quaternion = new Quaternion32();
      quaternion.setToNaN();

      Assert.assertTrue(Float.isNaN(quaternion.getX32()));
      Assert.assertTrue(Float.isNaN(quaternion.getY32()));
      Assert.assertTrue(Float.isNaN(quaternion.getZ32()));
      Assert.assertTrue(Float.isNaN(quaternion.getS32()));
   }

   @Test
   public void testContainsNaN()
   {
      Quaternion32 quaternion = new Quaternion32();

      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      Assert.assertFalse(quaternion.containsNaN());

      quaternion.setUnsafe(Float.NaN, 0.0, 0.0, 0.0);
      Assert.assertTrue(quaternion.containsNaN());
      
      quaternion.setUnsafe(0.0, Float.NaN, 0.0, 0.0);
      Assert.assertTrue(quaternion.containsNaN());
      
      quaternion.setUnsafe(0.0, 0.0, Float.NaN, 0.0);
      Assert.assertTrue(quaternion.containsNaN());
      
      quaternion.setUnsafe(0.0, 0.0, 0.0, Float.NaN);
      Assert.assertTrue(quaternion.containsNaN());
   }

   @Test
   public void testInterpolate()
   {
      Random random = new Random(6464L);
      Quaternion32 quaternion, quaternionCopy;
      Quaternion32 quaternion2, quaternion2Copy;
      Quaternion32 interpolated = new Quaternion32();
      Quaternion32 expected = new Quaternion32();
      float alpha;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test interpolate (Quaternion32Basics q1, float alpha)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         alpha = random.nextFloat();

         interpolated.interpolate(quaternion, alpha);
         QuaternionTools.interpolate(expected, quaternion, alpha, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(interpolated, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test interpolate (Quaternion32Basics q1, Quaternion32Basics q2, float alpha)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion2 = quaternion2Copy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         alpha = random.nextFloat();

         interpolated.interpolate(quaternion, quaternion2, alpha);
         QuaternionTools.interpolate(quaternion, quaternion2, alpha, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(interpolated, expected, EPS);
      }
   }

   @Test
   public void testInverse()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, expected = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         expected.set(quaternion);

         // corrupt
         quaternion.setUnsafe(quaternion.getX() + EPS, quaternion.getY() + EPS, quaternion.getZ() + EPS, quaternion.getS() + EPS);
         expected.setUnsafe(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());

         { // Test inverse()
            quaternion.inverse();
            expected.conjugate();
            expected.normalize();
            float norm = (float) Math.sqrt(quaternion.normSquared());
            float expectedNorm = (float) Math.sqrt(expected.normSquared());

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);

            Assert.assertEquals(norm, 1.0, EPS);
            Assert.assertEquals(expectedNorm, 1.0, EPS);
         }

         { // Test inverse(Quaternion32Basics other)
            quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
            expected.set(quaternion);

            Quaternion32 quaternion2 = new Quaternion32();
            Quaternion32 expectedQuaternion322 = new Quaternion32();

            quaternion2.setAndInverse(quaternion);
            expectedQuaternion322.conjugate();
            expectedQuaternion322.normalize();

            float norm2 = (float) Math.sqrt(quaternion2.normSquared());
            float expectedNorm2 = (float) Math.sqrt(expectedQuaternion322.normSquared());

            Assert.assertEquals(norm2, 1.0, EPS);
            Assert.assertEquals(expectedNorm2, 1.0, EPS);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);

            Assert.assertEquals(-quaternion2.getX(), quaternion.getX(), EPS);
            Assert.assertEquals(-quaternion2.getY(), quaternion.getY(), EPS);
            Assert.assertEquals(-quaternion2.getZ(), quaternion.getZ(), EPS);
            Assert.assertEquals( quaternion2.getS(), quaternion.getS(), EPS);
         }
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy;
      Quaternion32 quaternion2, quaternion2Copy;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion2 = quaternion2Copy = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         double dotProduct = quaternion.dot(quaternion2);
         double dot11 = quaternion.dot(quaternion);
         double dot22 = quaternion2.dot(quaternion2);

         double expected = Tuple4DTools.dot(quaternion, quaternion2);

         Assert.assertEquals(dotProduct, expected, EPS);
         Assert.assertEquals(dot11, 1, EPS);
         Assert.assertEquals(dot22, 1, EPS);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
      }
   }

   @Test
   public void testMultiply()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy = new Quaternion32();
      Quaternion32 quaternion2, quaternion2Copy = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion2Copy.set(quaternion2);

         { // Test multiply(QuaternionBasics other)
            quaternion.multiply(quaternion2);
            QuaternionTools.multiply(quaternionCopy, quaternion2, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }

         { // Test multiply(QuaternionBasics q1, QuaternionBasics q2)
            Quaternion32 product = new Quaternion32(), expected = new Quaternion32();
            product.multiply(quaternion, quaternion2);
            QuaternionTools.multiply(quaternionCopy, quaternion2, expected);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(product, expected, EPS);
         }

         { // Test multiply(Matrix3DReadOnly matrix)
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            quaternion.multiply(matrix);
            QuaternionTools.multiply(quaternionCopy, matrix, quaternionCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }
      }
   }

   @Test
   public void testMultiplyConjugate()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy = new Quaternion32();
      Quaternion32 quaternion2, quaternion2Copy = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion2Copy.set(quaternion2);

         { // Test multiplyConjugateThis(QuaternionBasics other)
            quaternion.multiplyConjugateThis(quaternion2);
            QuaternionTools.multiplyConjugateLeft(quaternionCopy, quaternion2, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }

         { // Test multiplyConjugateOther(QuaternionBasics other)
            quaternion.multiplyConjugateOther(quaternion2);
            QuaternionTools.multiplyConjugateRight(quaternionCopy, quaternion2, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }
      }
   }

   @Test
   public void testPreMultiply()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy = new Quaternion32();
      Quaternion32 quaternion2, quaternion2Copy = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion2Copy.set(quaternion2);

         { // Test preMultiply(QuaternionBasics other)
            quaternion.preMultiply(quaternion2);
            QuaternionTools.multiply(quaternion2, quaternionCopy, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }

         { // Test preMultiply(Matrix3DReadOnly matrix)
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            quaternion.preMultiply(matrix);
            QuaternionTools.multiply(matrix, quaternionCopy, quaternionCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }
      }
   }

   @Test
   public void testPreMultiplyConjugate()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion, quaternionCopy = new Quaternion32();
      Quaternion32 quaternion2, quaternion2Copy = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion2Copy.set(quaternion2);
         { // Test preMultiplyConjugateThis(QuaternionBasics other)
            quaternion.preMultiplyConjugateThis(quaternion2);
            QuaternionTools.multiplyConjugateRight(quaternion2, quaternionCopy, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }

         { // Test multiplyConjugateOther(QuaternionBasics other)
            quaternion.preMultiplyConjugateOther(quaternion2);
            QuaternionTools.multiplyConjugateLeft(quaternion2, quaternionCopy, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(65445L);
      Quaternion32 quaternion = new Quaternion32();
      Quaternion32 expected;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(QuaternionBasics other)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion.set(expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(float x, float y, float z, float s)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         quaternion.set(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32());

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(float[] quaternionArray)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         float[] quaternionArray, quaternionArrayCopy;
         quaternionArray = quaternionArrayCopy = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32()};

         quaternion.set(quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);

         Assert.assertEquals(quaternionArray[0], quaternion.getX(), EPS);
         Assert.assertEquals(quaternionArray[1], quaternion.getY(), EPS);
         Assert.assertEquals(quaternionArray[2], quaternion.getZ(), EPS);
         Assert.assertEquals(quaternionArray[3], quaternion.getS(), EPS);

         for (int j = 0; j < quaternionArray.length; j++)
            Assert.assertTrue(quaternionArray[j] == quaternionArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(float[] quaternionArray, int startIndex)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         float[] quaternionArray, quaternionArrayCopy;
         quaternionArray = quaternionArrayCopy = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getS32()};

         int startIndex, startIndexCopy;
         startIndex = startIndexCopy = 0;

         quaternion.set(quaternionArray, startIndex);

         Assert.assertEquals(quaternionArray[0], quaternion.getX(), EPS);
         Assert.assertEquals(quaternionArray[1], quaternion.getY(), EPS);
         Assert.assertEquals(quaternionArray[2], quaternion.getZ(), EPS);
         Assert.assertEquals(quaternionArray[3], quaternion.getS(), EPS);

         for (int j = 0; j < quaternionArray.length; j++)
            Assert.assertTrue(quaternionArray[j] == quaternionArrayCopy[j]);

         Assert.assertTrue(startIndex >= 0);
         Assert.assertTrue(startIndex == startIndexCopy);
         Assert.assertTrue(startIndex <= startIndex + quaternionArray.length);
      }

      Quaternion32 expectedQuaternion = new Quaternion32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {// Test set(AxisAngleBasics axisAngle)
         AxisAngle axisAngle, axisAngleCopy;
         axisAngle = axisAngleCopy = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

         quaternion.set(axisAngle);
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, (QuaternionBasics) expectedQuaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expectedQuaternion, EPS);
         GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngleCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(VectorBasics rotationVector)
         Vector rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         quaternion = new Quaternion32(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion((VectorReadOnly) rotationVector, (QuaternionBasics) expectedQuaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expectedQuaternion, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(2, 1);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());
         DenseMatrix64F denseMatrixCopy = new DenseMatrix64F(denseMatrix);

         try
         {
            quaternion.set(denseMatrix);
            fail("Should have thrown a IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IllegalArgumentException.");
         }

         Quaternion blop = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            assertTrue(denseMatrix.get(index) == denseMatrixCopy.get(index));

         denseMatrix = new DenseMatrix64F(5, 4);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());
         denseMatrix.set(0, 0, blop.getX());
         denseMatrix.set(1, 0, blop.getY());
         denseMatrix.set(2, 0, blop.getZ());
         denseMatrix.set(3, 0, blop.getS());
         denseMatrixCopy = new DenseMatrix64F(denseMatrix);
         quaternion.set(denseMatrix);

         assertEquals(quaternion.getX(), denseMatrix.get(0, 0), EPS);
         assertEquals(quaternion.getY(), denseMatrix.get(1, 0), EPS);
         assertEquals(quaternion.getZ(), denseMatrix.get(2, 0), EPS);
         assertEquals(quaternion.getS(), denseMatrix.get(3, 0), EPS);

         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            assertTrue(denseMatrix.get(index) == denseMatrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix, int startRow);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 1);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());
         DenseMatrix64F denseMatrixCopy = new DenseMatrix64F(denseMatrix);

         try
         {
            quaternion.set(denseMatrix, 2);
            fail("Should have thrown a IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IllegalArgumentException.");
         }
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            assertTrue(denseMatrix.get(index) == denseMatrixCopy.get(index));

         Quaternion blop = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         denseMatrix = new DenseMatrix64F(10, 4);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());
         denseMatrix.set(5, 0, blop.getX());
         denseMatrix.set(6, 0, blop.getY());
         denseMatrix.set(7, 0, blop.getZ());
         denseMatrix.set(8, 0, blop.getS());
         denseMatrixCopy = new DenseMatrix64F(denseMatrix);
         quaternion.set(denseMatrix, 5);

         assertEquals(quaternion.getX(), denseMatrix.get(5, 0), EPS);
         assertEquals(quaternion.getY(), denseMatrix.get(6, 0), EPS);
         assertEquals(quaternion.getZ(), denseMatrix.get(7, 0), EPS);
         assertEquals(quaternion.getS(), denseMatrix.get(8, 0), EPS);

         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            assertTrue(denseMatrix.get(index) == denseMatrixCopy.get(index));
      }
   }

   @Test
   public void testTransform()
   {
      Random random = new Random(6787L);
      Quaternion32 quaternion = new Quaternion32();

      Vector tuple, tupleCopy = new Vector();
      Vector transform3D = new Vector();
      Vector expectedTransform3D = new Vector();

      Vector2D tuple2D, tuple2DCopy = new Vector2D();
      Vector2D transform2D = new Vector2D();
      Vector2D expectedTransform2D = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleToTransform)
         tuple = GeometryBasicsRandomTools.generateRandomVector(random);
         tupleCopy.set(tuple);
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         quaternion.transform((TupleBasics) tuple);
         QuaternionTools.transform(quaternion, tupleCopy);

         GeometryBasicsTestTools.assertRotationVectorEquals(tuple, tupleCopy, QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleOriginal, TupleBasics tupleTransformed)
         tuple = GeometryBasicsRandomTools.generateRandomVector(random);
         tupleCopy.set(tuple);
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         quaternion.transform((TupleBasics) tuple, transform3D);
         QuaternionTools.transform(quaternion, tupleCopy, expectedTransform3D);

         GeometryBasicsTestTools.assertRotationVectorEquals(tuple, tupleCopy, QuaternionTest.EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(transform3D, expectedTransform3D, QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.transform((Tuple2DBasics) tuple2D, (Tuple2DBasics) transform2D);
         QuaternionTools.transform(quaternion, tuple2DCopy, expectedTransform2D, false);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(transform2D.getX(), expectedTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(transform2D.getY(), expectedTransform2D.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.transform((Tuple2DBasics) tuple2D, (Tuple2DBasics) transform2D);
         QuaternionTools.transform(quaternion, tuple2DCopy, expectedTransform2D, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(transform2D.getX(), expectedTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(transform2D.getY(), expectedTransform2D.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         quaternion.transform((Tuple2DBasics) tuple2D, false);
         QuaternionTools.transform(quaternion, tuple2DCopy, false);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.transform((Tuple2DBasics) tuple2D, true);
         QuaternionTools.transform(quaternion, tuple2DCopy, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.transform((Tuple2DBasics) tuple2D, (Tuple2DBasics) transform2D);
         QuaternionTools.transform(quaternion, tuple2DCopy, expectedTransform2D, false);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(transform2D.getX(), expectedTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(transform2D.getY(), expectedTransform2D.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.transform((Tuple2DBasics) tuple2D, (Tuple2DBasics) transform2D);
         QuaternionTools.transform(quaternion, tuple2DCopy, expectedTransform2D, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(transform2D.getX(), expectedTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(transform2D.getY(), expectedTransform2D.getY(), QuaternionTest.EPS);
      }
   }

   @Test
   public void testInverseTransform()
   {
      Random random = new Random(6787L);
      Quaternion32 quaternion = new Quaternion32();

      Vector tuple, tupleCopy = new Vector();
      Vector inverseTransform3D = new Vector();
      Vector expectedInverseTransform3D = new Vector();

      Vector2D tuple2D, tuple2DCopy = new Vector2D();
      Vector2D inverseTransform2D = new Vector2D();
      Vector2D expectedInverseTransform2D = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(TupleBasics tupleToInverseTransform)
         tuple = GeometryBasicsRandomTools.generateRandomVector(random);
         tupleCopy.set(tuple);
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         quaternion.inverseTransform((TupleBasics) tuple);
         QuaternionTools.inverseTransform(quaternion, tupleCopy);

         GeometryBasicsTestTools.assertRotationVectorEquals(tuple, tupleCopy, QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(TupleBasics tupleOriginal, TupleBasics tupleInverseTransformed)
         tuple = GeometryBasicsRandomTools.generateRandomVector(random);
         tupleCopy.set(tuple);
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         quaternion.inverseTransform((TupleBasics) tuple, inverseTransform3D);
         QuaternionTools.inverseTransform(quaternion, tuple, expectedInverseTransform3D);

         GeometryBasicsTestTools.assertRotationVectorEquals(tuple, tupleCopy, QuaternionTest.EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(inverseTransform3D, expectedInverseTransform3D, QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleInverseTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.inverseTransform((Tuple2DBasics) tuple2D, (Tuple2DBasics) inverseTransform2D);
         QuaternionTools.inverseTransform(quaternion, tuple2DCopy, expectedInverseTransform2D, false);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(inverseTransform2D.getX(), expectedInverseTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(inverseTransform2D.getY(), expectedInverseTransform2D.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleInverseTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.inverseTransform((Tuple2DBasics) tuple2D, (Tuple2DBasics) inverseTransform2D);
         QuaternionTools.inverseTransform(quaternion, tuple2DCopy, expectedInverseTransform2D, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(inverseTransform2D.getX(), expectedInverseTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(inverseTransform2D.getY(), expectedInverseTransform2D.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleToInverseTransform)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.inverseTransform((Tuple2DBasics) tuple2D);
         QuaternionTools.inverseTransform(quaternion, tuple2DCopy, false);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleToInverseTransform)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.inverseTransform((Tuple2DBasics) tuple2D);
         QuaternionTools.inverseTransform(quaternion, tuple2DCopy, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleInverseTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.inverseTransform((Tuple2DBasics) tuple2D, (Tuple2DBasics) inverseTransform2D);
         QuaternionTools.inverseTransform(quaternion, tuple2DCopy, expectedInverseTransform2D, false);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(inverseTransform2D.getX(), expectedInverseTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(inverseTransform2D.getY(), expectedInverseTransform2D.getY(), QuaternionTest.EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleInverseTransformed)
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = new Quaternion32(0.0f, 0.0f, qz, qs);

         quaternion.inverseTransform((Tuple2DBasics) tuple2D, (Tuple2DBasics) inverseTransform2D);
         QuaternionTools.inverseTransform(quaternion, tuple2DCopy, expectedInverseTransform2D, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), QuaternionTest.EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), QuaternionTest.EPS);

         Assert.assertEquals(inverseTransform2D.getX(), expectedInverseTransform2D.getX(), QuaternionTest.EPS);
         Assert.assertEquals(inverseTransform2D.getY(), expectedInverseTransform2D.getY(), QuaternionTest.EPS);
      }
   }

   @Test
   @Ignore
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetAngle()
   {
      // TODO reimplement me
   }

   @Test
   public void testGet()
   {
      Random random = new Random(234234L);
      Quaternion32 expected;
      Quaternion32 quaternion = new Quaternion32();
      float[] quaternionArray = new float[4];

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(float[] quaternionArray)
         expected = quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         expected.get(quaternionArray);

         Assert.assertTrue(quaternionArray[0] == quaternion.getX());
         Assert.assertTrue(quaternionArray[1] == quaternion.getY());
         Assert.assertTrue(quaternionArray[2] == quaternion.getZ());
         Assert.assertTrue(quaternionArray[3] == quaternion.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(float[] quaternionArray, int startIndex)
         expected = quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

         int startIndex;
         int startIndexCopy;
         startIndex = startIndexCopy = 0;

         quaternion.get(quaternionArray, startIndex);

         Assert.assertTrue(quaternionArray[0] == quaternion.getX());
         Assert.assertTrue(quaternionArray[1] == quaternion.getY());
         Assert.assertTrue(quaternionArray[2] == quaternion.getZ());
         Assert.assertTrue(quaternionArray[3] == quaternion.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);

         Assert.assertTrue(startIndex == startIndexCopy);
         Assert.assertTrue(startIndex >= 0);
         Assert.assertTrue(startIndex <= startIndex + quaternionArray.length);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(Quaternion32Basics quaternionToPack)
         quaternion = expected = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         Quaternion32 quaternion2 = new Quaternion32();

         quaternion.get((QuaternionBasics) quaternion2);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
      }
   }

   @Test
   public void testGetDenseMatrix()
   {
      Random random = new Random(57684L);
      Quaternion32 q = new Quaternion32();

      { // Test get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F denseMatrix = new DenseMatrix64F(2, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         try
         {
            q.get(denseMatrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }

         denseMatrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         q = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         q.get(denseMatrix);
         assertEquals(q.getX(), denseMatrix.get(0, 0), 1.0e-7);
         assertEquals(q.getY(), denseMatrix.get(1, 0), 1.0e-7);
         assertEquals(q.getZ(), denseMatrix.get(2, 0), 1.0e-7);
         assertEquals(q.getS(), denseMatrix.get(3, 0), 1.0e-7);
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack, int startRow)
         DenseMatrix64F denseMatrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         try
         {
            q.get(denseMatrix, 4);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }

         denseMatrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         q = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         q.get(denseMatrix, 2);
         assertEquals(q.getX(), denseMatrix.get(2, 0), 1.0e-7);
         assertEquals(q.getY(), denseMatrix.get(3, 0), 1.0e-7);
         assertEquals(q.getZ(), denseMatrix.get(4, 0), 1.0e-7);
         assertEquals(q.getS(), denseMatrix.get(5, 0), 1.0e-7);
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack, int startRow, int column)
         DenseMatrix64F denseMatrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         try
         {
            q.get(denseMatrix, 4, 3);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }

         denseMatrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         q = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         q.get(denseMatrix, 2, 4);
         assertEquals(q.getX(), denseMatrix.get(2, 4), 1.0e-7);
         assertEquals(q.getY(), denseMatrix.get(3, 4), 1.0e-7);
         assertEquals(q.getZ(), denseMatrix.get(4, 4), 1.0e-7);
         assertEquals(q.getS(), denseMatrix.get(5, 4), 1.0e-7);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      // TODO reimplement me
   }

   @Test
   public void testEquals() throws Exception
   {
      // TODO reimplement me
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion32 q = GeometryBasicsRandomTools.generateRandomQuaternion32(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float qx = q.getX32();
         float qy = q.getY32();
         float qz = q.getZ32();
         float qs = q.getS32();
         switch (random.nextInt(4))
         {
         case 0: qx = random.nextFloat(); break;
         case 1: qy = random.nextFloat(); break;
         case 2: qz = random.nextFloat(); break;
         case 3: qs = random.nextFloat(); break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
