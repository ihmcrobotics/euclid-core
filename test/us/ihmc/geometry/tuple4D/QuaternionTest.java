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
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public class QuaternionTest extends QuaternionBasicsTest<Quaternion>
{
   public static final int NUMBER_OF_ITERATIONS = 100;
   public static final double EPS = 1e-14;

   @Test
   public void testQuaternion()
   {
      Random random = new Random(613615L);
      Quaternion quaternion = new Quaternion();
      Quaternion quaternionCopy;
      Quaternion expected = new Quaternion();

      { // Test Quaternion()
         expected.setToZero();
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(QuaternionBasics other)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Quaternion quaternion2 = new Quaternion((QuaternionReadOnly<?>) quaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         expected.normalizeAndLimitToPiMinusPi();
         quaternion = new Quaternion(expected.getX(), expected.getY(), expected.getZ(), expected.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(double[] quaternionArray)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         double[] quaternionArray;
         quaternionArray = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getS()};

         quaternion = new Quaternion(quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(RotationMatrix rotationMatrix)
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         quaternion = new Quaternion(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, (QuaternionBasics<?>) expected);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(VectorBasics rotationVector)
         Vector3D rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         quaternion = new Quaternion(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion((Vector3DReadOnly<?>) rotationVector, (QuaternionBasics<?>) expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
      }
   }

   @Test
   public void testConjugate()
   {
      Random random = new Random(65445L);
      Quaternion quaternion, quaternionCopy = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternionCopy.set(quaternion);

         { // Test conjugate()
            quaternion.conjugate();

            Assert.assertEquals(quaternion.getX(), -quaternionCopy.getX(), EPS);
            Assert.assertEquals(quaternion.getY(), -quaternionCopy.getY(), EPS);
            Assert.assertEquals(quaternion.getZ(), -quaternionCopy.getZ(), EPS);
            Assert.assertEquals(quaternion.getS(), quaternionCopy.getS(), EPS);
         }

         { // Test conjugate (QuaternionBasics other)
            Quaternion quaternion2 = new Quaternion();
            quaternion2.setAndConjugate(quaternionCopy);

            Assert.assertTrue(quaternion2.getX() == -quaternionCopy.getX());
            Assert.assertTrue(quaternion2.getY() == -quaternionCopy.getY());
            Assert.assertTrue(quaternion2.getZ() == -quaternionCopy.getZ());
            Assert.assertTrue(quaternion2.getS() == quaternionCopy.getS());
         }
      }
   }

   @Test
   public void testNegate()
   {
      Random random = new Random(65445L);
      Quaternion quaternion, expected;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         { // Test negate()
            quaternion = new Quaternion(expected.getX(), expected.getY(), expected.getZ(), expected.getS());

            quaternion.negate();

            Assert.assertEquals(expected.getX(), -quaternion.getX(), EPS);
            Assert.assertEquals(expected.getY(), -quaternion.getY(), EPS);
            Assert.assertEquals(expected.getZ(), -quaternion.getZ(), EPS);
            Assert.assertEquals(expected.getS(), -quaternion.getS(), EPS);
         }

         { // Test negate (QuaternionBasics other)
            quaternion = new Quaternion(expected.getX(), expected.getY(), expected.getZ(), expected.getS());

            Quaternion quaternion2 = new Quaternion();
            quaternion2.setAndNegate(quaternion);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);

            Assert.assertEquals(quaternion2.getX(), -quaternion.getX(), EPS);
            Assert.assertEquals(quaternion2.getY(), -quaternion.getY(), EPS);
            Assert.assertEquals(quaternion2.getZ(), -quaternion.getZ(), EPS);
            Assert.assertEquals(quaternion2.getS(), -quaternion.getS(), EPS);
         }
      }
   }

   @Test
   public void testNormalize()
   {
      Random random = new Random(15461L);

      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

      double sinHalfTheta = Math.sin(theta / 2.0);
      double cosHalfTheta = Math.cos(theta / 2.0);

      double qx = axis.getX() * sinHalfTheta;
      double qy = axis.getY() * sinHalfTheta;
      double qz = axis.getZ() * sinHalfTheta;
      double qs = cosHalfTheta;

      // Test that it does not mess up a quaternion already normalized
      Quaternion qExpected = new Quaternion();
      qExpected.setUnsafe(qx, qy, qz, qs);
      Quaternion qActual = new Quaternion();
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.normalize();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that the quaternion is normalized
      double scale = random.nextDouble();
      qActual.setUnsafe(scale * qx, scale * qy, scale * qz, scale * qs);
      qActual.normalize();

      assertEquals(1.0, QuaternionTools.norm(qActual), EPS);
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that the quaternion is not kept within [-Pi, Pi]
      theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI, 2.0 * Math.PI);
      sinHalfTheta = Math.sin(theta / 2.0);
      cosHalfTheta = Math.cos(theta / 2.0);

      qx = axis.getX() * sinHalfTheta;
      qy = axis.getY() * sinHalfTheta;
      qz = axis.getZ() * sinHalfTheta;
      qs = cosHalfTheta;

      qExpected.setUnsafe(qx, qy, qz, qs);
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.normalize();
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
   }

   @Test
   public void testNormalizeAndLimitToPiMinusPi()
   {
      Random random = new Random(15461L);

      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

      double sinHalfTheta = Math.sin(theta / 2.0);
      double cosHalfTheta = Math.cos(theta / 2.0);

      double qx = axis.getX() * sinHalfTheta;
      double qy = axis.getY() * sinHalfTheta;
      double qz = axis.getZ() * sinHalfTheta;
      double qs = cosHalfTheta;

      // Test that it does not mess up a quaternion already normalized
      Quaternion qExpected = new Quaternion();
      qExpected.setUnsafe(qx, qy, qz, qs);
      Quaternion qActual = new Quaternion();
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.normalizeAndLimitToPiMinusPi();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that the quaternion is normalized
      double scale = random.nextDouble();
      qActual.setUnsafe(scale * qx, scale * qy, scale * qz, scale * qs);
      qActual.normalizeAndLimitToPiMinusPi();

      assertEquals(1.0, QuaternionTools.norm(qActual), EPS);
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that the quaternion is not kept within [-Pi, Pi]
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI, 2.0 * Math.PI);
         sinHalfTheta = Math.sin(theta / 2.0);
         cosHalfTheta = Math.cos(theta / 2.0);

         qx = axis.getX() * sinHalfTheta;
         qy = axis.getY() * sinHalfTheta;
         qz = axis.getZ() * sinHalfTheta;
         qs = cosHalfTheta;

         qExpected.setUnsafe(qx, qy, qz, qs);
         qActual.setUnsafe(qx, qy, qz, qs);
         qActual.normalizeAndLimitToPiMinusPi();
         if (Math.abs(qExpected.getAngle()) < Math.PI)
         {
            GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
         }
         else
         {
            assertTrue(Math.abs(qActual.getAngle()) < Math.PI);
            qExpected.negate();
            GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
         }
      }
   }

   @Test
   public void testIsNormalized()
   {
      Random random = new Random(65445L);
      Quaternion quaternion, quaternionCopy;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         double normSquared = quaternion.lengthSquared();
         Assert.assertTrue(quaternion.isNormalized(EPS)); // Quaternion should have norm = 1
         Assert.assertFalse(Double.isNaN(normSquared));
         Assert.assertTrue(Math.abs(normSquared - 1.0) < EPS);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternionCopy, quaternionCopy, EPS);
      }
   }

   @Test
   public void testSetToZero()
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setToZero();
      Quaternion zeroQ = new Quaternion(0.0, 0.0, 0.0, 1.0);

      GeometryBasicsTestTools.assertQuaternionEquals(quaternion, zeroQ, EPS);
   }

   @Test
   public void testSetToNaN()
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setToNaN();

      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(quaternion);
   }

   @Test
   public void testInterpolate()
   {
      Random random = new Random(6464L);
      Quaternion quaternion, quaternionCopy;
      Quaternion quaternion2, quaternion2Copy;
      Quaternion interpolated = new Quaternion();
      Quaternion expected = new Quaternion();
      double alpha;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test interpolate (QuaternionBasics q1, double alpha)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         alpha = random.nextDouble();

         interpolated.interpolate(quaternion, alpha);
         QuaternionTools.interpolate(expected, quaternion, alpha, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(interpolated, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test interpolate (QuaternionBasics q1, QuaternionBasics q2, double alpha)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion2 = quaternion2Copy = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         alpha = random.nextDouble();

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
      Random random = new Random(15461L);

      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

      double sinHalfTheta = Math.sin(theta / 2.0);
      double cosHalfTheta = Math.cos(theta / 2.0);

      double qx = axis.getX() * sinHalfTheta;
      double qy = axis.getY() * sinHalfTheta;
      double qz = axis.getZ() * sinHalfTheta;
      double qs = cosHalfTheta;

      // Test that it computes the inverse
      Quaternion qExpected = new Quaternion();
      qExpected.setUnsafe(-qx, -qy, -qz, qs);
      Quaternion qActual = new Quaternion();
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.inverse();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that the quaternion is normalized
      double scale = random.nextDouble();
      qActual.setUnsafe(scale * qx, scale * qy, scale * qz, scale * qs);
      qActual.inverse();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that the quaternion is not kept within [-Pi, Pi]
      theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI, 2.0 * Math.PI);
      sinHalfTheta = Math.sin(theta / 2.0);
      cosHalfTheta = Math.cos(theta / 2.0);

      qx = axis.getX() * sinHalfTheta;
      qy = axis.getY() * sinHalfTheta;
      qz = axis.getZ() * sinHalfTheta;
      qs = cosHalfTheta;

      qExpected.setUnsafe(-qx, -qy, -qz, qs);
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.inverse();
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      // Test that setAndInverse() does "set" and "inverse"
      Quaternion qOriginal = GeometryBasicsRandomTools.generateRandomQuaternion(random);
      qExpected.set(qOriginal);
      qExpected.inverse();
      qActual.setAndInverse(qOriginal);
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
   }

   @Test
   public void testDifference()
   {
      Random random = new Random(65445L);
      Quaternion quaternion, quaternionCopy;
      Quaternion quaternion2, quaternion2Copy;
      Quaternion diff = new Quaternion();
      Quaternion expected = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion2 = quaternion2Copy = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         diff.difference(quaternion, quaternion2);
         QuaternionTools.multiplyConjugateLeft(quaternion, quaternion2, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(diff, expected, EPS);
      }
   }

   @Test
   public void testMultiply()
   {
      Random random = new Random(65445L);
      Quaternion quaternion, quaternionCopy = new Quaternion();
      Quaternion quaternion2, quaternion2Copy = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion2Copy.set(quaternion2);

         { // Test multiply(QuaternionBasics other)
            quaternion.multiply(quaternion2);
            QuaternionTools.multiply(quaternionCopy, quaternion2Copy, quaternionCopy);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion2, quaternion2Copy, EPS);
         }

         { // Test multiply(QuaternionBasics q1, QuaternionBasics q2)
            Quaternion product = new Quaternion(), expected = new Quaternion();
            product.multiply(quaternion, quaternion2);
            QuaternionTools.multiply(quaternionCopy, quaternion2Copy, expected);

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
      Quaternion quaternion, quaternionCopy = new Quaternion();
      Quaternion quaternion2, quaternion2Copy = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion(random);
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
      Quaternion quaternion, quaternionCopy = new Quaternion();
      Quaternion quaternion2, quaternion2Copy = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion(random);
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
      Quaternion quaternion, quaternionCopy = new Quaternion();
      Quaternion quaternion2, quaternion2Copy = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternionCopy.set(quaternion);
         quaternion2 = GeometryBasicsRandomTools.generateRandomQuaternion(random);
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
      Quaternion quaternion = new Quaternion();
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();
      RotationMatrix rotationMatrix = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(QuaternionBasics other)
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, 2.0 * Math.PI);

         // corrupt
         qActual.setUnsafe(quaternion.getX() + EPS, quaternion.getY() + EPS, quaternion.getZ() + EPS, quaternion.getS() + EPS);
         qExpected.setUnsafe(quaternion.getX() + EPS, quaternion.getY() + EPS, quaternion.getZ() + EPS, quaternion.getS() + EPS);

         qActual.set(qActual);
         qExpected.normalize();

         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(double x, double y, double z, double s)
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, 2.0 * Math.PI);

         // corrupt
         qActual.set(quaternion.getX() + EPS, quaternion.getY() + EPS, quaternion.getZ() + EPS, quaternion.getS() + EPS);

         qExpected.setUnsafe(quaternion.getX() + EPS, quaternion.getY() + EPS, quaternion.getZ() + EPS, quaternion.getS() + EPS);
         qExpected.normalize();

         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(double[] quaternionArray)
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, 2.0 * Math.PI);
         qExpected.setUnsafe(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
         double[] quaternionArray = {quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS()};

         qActual.set(quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(double[] quaternionArray, int startIndex)
         qExpected = GeometryBasicsRandomTools.generateRandomQuaternion(random, 2.0 * Math.PI);

         int startIndex = random.nextInt(10);
         double[] quaternionArray = new double[4 + startIndex];
         for (int index = startIndex; index < startIndex + 4; index++)
            quaternionArray[index] = qExpected.get(index - startIndex);

         qActual.set(startIndex, quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {// Test set(AxisAngleBasics axisAngle)
         AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
         qActual.set(axisAngle);
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, qExpected);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(RotationMatrix rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         qActual.set(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, qExpected);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(VectorBasics rotationVector)
         Vector3D rotationVector = GeometryBasicsRandomTools.generateRandomRotationVector(random);
         qActual.set(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, qExpected);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix)
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, 2.0 * Math.PI);
         qExpected.setUnsafe(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
         DenseMatrix64F qDenseMatrix = new DenseMatrix64F(4, 1);
         qDenseMatrix.set(4, 1, true, quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());

         qActual.set(qDenseMatrix);

         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(DenseMatrix64F matrix, int startRow)
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, 2.0 * Math.PI);
         qExpected.setUnsafe(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
         int startRow = random.nextInt(10);
         DenseMatrix64F qDenseMatrix = new DenseMatrix64F(4 + startRow, 1);
         for (int row = startRow; row < startRow + 4; row++)
            qDenseMatrix.set(row, 0, quaternion.get(row - startRow));

         qActual.set(startRow, qDenseMatrix);

         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
      }
   }

   @Test
   public void testSetYawPitchRoll()
   {
      Random random = new Random(574631L);
      Quaternion actualQuaternion = new Quaternion();
      Quaternion expectedQuaternion = new Quaternion();
      double[] yawPitchRoll;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setYawPitchRoll(double[] yawPitchRoll)
         yawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);

         QuaternionConversion.convertYawPitchRollToQuaternion(yawPitchRoll, expectedQuaternion);

         actualQuaternion.setYawPitchRoll(yawPitchRoll);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);

         actualQuaternion.setToZero();
         actualQuaternion.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
      }
   }

   @Test
   public void testTransformTuple()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      Vector3D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector3D(random);
      Vector3D vectorExpected = new Vector3D();
      Vector3D vectorActual = new Vector3D();

      QuaternionTools.transform(quaternion, vectorOriginal, vectorExpected);

      quaternion.transform(vectorOriginal, vectorActual);
      GeometryBasicsTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPS);

      vectorActual.set(vectorOriginal);
      quaternion.transform(vectorActual);
      GeometryBasicsTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testTransformTuple2D()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = new Quaternion();
      double yaw = 2.0 * Math.PI * random.nextDouble();
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, 0.0, 0.0, quaternion);

      Vector2D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector2D(random);
      Vector2D vectorExpected = new Vector2D();
      Vector2D vectorActual = new Vector2D();

      QuaternionTools.transform(quaternion, vectorOriginal, vectorExpected, true);

      quaternion.transform(vectorOriginal, vectorActual);
      GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);

      quaternion.transform(vectorOriginal, vectorActual, true);
      GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);

      vectorActual.set(vectorOriginal);
      quaternion.transform(vectorActual, true);
      GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);

      try
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.transform(vectorActual, true);
         fail("Shoudl have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.transform(vectorOriginal, vectorActual, true);
         fail("Shoudl have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.transform(vectorOriginal, vectorActual);
         fail("Shoudl have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testTransformQuaternion()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      Quaternion qOriginal = GeometryBasicsRandomTools.generateRandomQuaternion(random);
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();

      qExpected.multiply(quaternion, qOriginal);

      quaternion.transform(qOriginal, qActual);
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);

      qActual.set(qOriginal);
      quaternion.transform(qActual);
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, EPS);
   }

   @Test
   public void testTransformVector4D()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      Vector4D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D vectorExpected = new Vector4D();
      Vector4D vectorActual = new Vector4D();

      QuaternionTools.transform(quaternion, vectorOriginal, vectorExpected);

      quaternion.transform(vectorOriginal, vectorActual);
      GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);

      vectorActual.set(vectorOriginal);
      quaternion.transform(vectorActual);
      GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testTransformMatrix3D()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      Matrix3D matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

      quaternion.transform(matrixOriginal, matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.set(matrixOriginal);
      quaternion.transform(matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testTransformRotationMatrix()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      RotationMatrix matrixOriginal = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

      quaternion.transform(matrixOriginal, matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.set(matrixOriginal);
      quaternion.transform(matrixActual);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testInverseTransformTuple()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = new Quaternion();

      Vector3D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector3D(random);
      Vector3D vectorTransformed = new Vector3D();
      Vector3D vectorExpected = new Vector3D(vectorOriginal);
      Vector3D vectorActual = new Vector3D();

      quaternion.transform(vectorOriginal, vectorTransformed);
      quaternion.inverseTransform(vectorTransformed, vectorActual);
      GeometryBasicsTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPS);

      quaternion.transform(vectorOriginal, vectorActual);
      quaternion.inverseTransform(vectorActual);
      GeometryBasicsTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testInverseTransformTuple2D()
   {
      Random random = new Random(6787L);
      Quaternion quaternion = new Quaternion();
      double yaw = 2.0 * Math.PI * random.nextDouble();
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, 0.0, 0.0, quaternion);

      Vector2D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector2D(random);
      Vector2D vectorTransformed = new Vector2D();
      Vector2D vectorExpected = new Vector2D(vectorOriginal);
      Vector2D vectorActual = new Vector2D();

      quaternion.transform(vectorOriginal, vectorTransformed);
      quaternion.inverseTransform(vectorTransformed, vectorActual);
      GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);

      quaternion.transform(vectorOriginal, vectorActual);
      quaternion.inverseTransform(vectorActual);
      GeometryBasicsTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);

      try
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.inverseTransform(vectorActual);
         fail("Shoudl have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.inverseTransform(vectorOriginal, vectorActual);
         fail("Shoudl have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
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
      Random random = new Random(65445L);
      double expectedAngle = 2.0 * Math.PI * random.nextDouble(); // Sign issue when theta < 0.0
      double c = Math.cos(expectedAngle / 2.0);
      double s = Math.sin(expectedAngle / 2.0);
      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      Quaternion q = new Quaternion();
      double qx = s * axis.getX();
      double qy = s * axis.getY();
      double qz = s * axis.getZ();
      double qs = c;
      q.setUnsafe(qx, qy, qz, qs);

      assertEquals(expectedAngle, q.getAngle(), EPS);
   }

   @Test
   public void testGet()
   {
      Random random = new Random(234234L);
      Quaternion expected;
      Quaternion quaternion = new Quaternion();
      double[] quaternionArray = new double[4];

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(double[] quaternionArray)
         expected = quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         expected.get(quaternionArray);

         Assert.assertTrue(quaternionArray[0] == quaternion.getX());
         Assert.assertTrue(quaternionArray[1] == quaternion.getY());
         Assert.assertTrue(quaternionArray[2] == quaternion.getZ());
         Assert.assertTrue(quaternionArray[3] == quaternion.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(double[] quaternionArray, int startIndex)
         expected = quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         int startIndex;
         int startIndexCopy;
         startIndex = startIndexCopy = 0;

         quaternion.get(startIndex, quaternionArray);

         Assert.assertTrue(quaternionArray[0] == quaternion.getX());
         Assert.assertTrue(quaternionArray[1] == quaternion.getY());
         Assert.assertTrue(quaternionArray[2] == quaternion.getZ());
         Assert.assertTrue(quaternionArray[3] == quaternion.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);

         Assert.assertTrue(startIndex == startIndexCopy);
         Assert.assertTrue(startIndex >= 0);
         Assert.assertTrue(startIndex <= startIndex + quaternionArray.length);
      }
   }

   @Test
   public void testGetDenseMatrix()
   {
      Random random = new Random(57684L);
      Quaternion quaternion = new Quaternion();

      { // Test get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F denseMatrix = new DenseMatrix64F(2, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         try
         {
            quaternion.get(denseMatrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }

         denseMatrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.get(denseMatrix);
         assertTrue(quaternion.getX() == denseMatrix.get(0, 0));
         assertTrue(quaternion.getY() == denseMatrix.get(1, 0));
         assertTrue(quaternion.getZ() == denseMatrix.get(2, 0));
         assertTrue(quaternion.getS() == denseMatrix.get(3, 0));
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack, int startRow)
         DenseMatrix64F denseMatrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         try
         {
            quaternion.get(4, denseMatrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }

         denseMatrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.get(2, denseMatrix);
         assertTrue(quaternion.getX() == denseMatrix.get(2, 0));
         assertTrue(quaternion.getY() == denseMatrix.get(3, 0));
         assertTrue(quaternion.getZ() == denseMatrix.get(4, 0));
         assertTrue(quaternion.getS() == denseMatrix.get(5, 0));
      }

      { // Test get(DenseMatrix64F tupleMatrixToPack, int startRow, int column)
         DenseMatrix64F denseMatrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         try
         {
            quaternion.get(4, 3, denseMatrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }

         denseMatrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < denseMatrix.getNumElements(); index++)
            denseMatrix.set(index, random.nextDouble());

         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion.get(2, 4, denseMatrix);
         assertTrue(quaternion.getX() == denseMatrix.get(2, 4));
         assertTrue(quaternion.getY() == denseMatrix.get(3, 4));
         assertTrue(quaternion.getZ() == denseMatrix.get(4, 4));
         assertTrue(quaternion.getS() == denseMatrix.get(5, 4));
      }
   }

   @Test
   public void testGetYawPitchRoll()
   {
      Random random = new Random(654651351L);
      Quaternion quaternion, quaternionCopy;
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         { // Test getYawPitchRoll(double[] yawPitchRollToPack)
            double[] yawPitchRoll = new double[4];
            quaternion.getYawPitchRoll(yawPitchRoll);
            double[] expectedYawPitchRoll = new double[4];
            YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, expectedYawPitchRoll);

            for (int j = 0; j < yawPitchRoll.length; j++)
               Assert.assertEquals(yawPitchRoll[j], expectedYawPitchRoll[j], EPS);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }

         { // Test getYaw()
            double yaw = quaternion.getYaw();
            double expectedYaw = YawPitchRollConversion.computeYaw(quaternion);
            Assert.assertEquals(yaw, expectedYaw, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }

         { // Test getPitch()
            double pitch = quaternion.getPitch();
            double expectedPitch = YawPitchRollConversion.computePitch(quaternion);
            Assert.assertEquals(pitch, expectedPitch, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }

         { // Test getRoll()
            double roll = quaternion.getRoll();
            double expectedRoll = YawPitchRollConversion.computeRoll(quaternion);
            Assert.assertEquals(roll, expectedRoll, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }
      }
   }

   @Test
   public void testGetWithIndex() throws Exception
   {
      Random random = new Random(654651351L);
      Quaternion q = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      assertTrue(q.get(0) == q.getX());
      assertTrue(q.get(1) == q.getY());
      assertTrue(q.get(2) == q.getZ());
      assertTrue(q.get(3) == q.getS());

      try
      {
         q.get(4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // good
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion q = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double qx = q.getX();
         double qy = q.getY();
         double qz = q.getZ();
         double qs = q.getS();
         switch (random.nextInt(4))
         {
         case 0:
            qx = random.nextDouble();
            break;
         case 1:
            qy = random.nextDouble();
            break;
         case 2:
            qz = random.nextDouble();
            break;
         case 3:
            qs = random.nextDouble();
            break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Quaternion createEmptyTuple()
   {
      return new Quaternion();
   }

   @Override
   public Quaternion createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomQuaternion(random);
   }

   @Override
   public Quaternion createTuple(double x, double y, double z, double s)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(x, y, z, s);
      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}
