package us.ihmc.geometry.rotationConversion;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.rotationConversion.AxisAngleConversion;
import us.ihmc.geometry.tools.EuclidCoreRandomTools;
import us.ihmc.geometry.tools.EuclidCoreTestTools;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;

public class AxisAngleConversionTest
{
   private static final double EPSILON = 1.0e-12;
   public static final int NUMBER_OF_ITERATIONS = 2000;

   @Test
   public void testQuaternionToAxisAngle() throws Exception
   {
      Random random = new Random(51651L);
      AxisAngle axisAngle = new AxisAngle();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double ux = EuclidCoreRandomTools.generateRandomDouble(random);
         double uy = EuclidCoreRandomTools.generateRandomDouble(random);
         double uz = EuclidCoreRandomTools.generateRandomDouble(random);

         double norm = Math.sqrt(ux * ux + uy * uy + uz * uz);
         ux /= norm;
         uy /= norm;
         uz /= norm;
         double angle = EuclidCoreRandomTools.generateRandomDouble(random, 2.0 * Math.PI);

         double qs = Math.cos(angle / 2.0);
         double qx = ux * Math.sin(angle / 2.0);
         double qy = uy * Math.sin(angle / 2.0);
         double qz = uz * Math.sin(angle / 2.0);
         quaternion.setUnsafe(qx, qy, qz, qs);
         AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);

         if (axisAngle.getAngle() * angle < 0.0)
         {
            axisAngle.negate();
         }

         assertEquals(ux, axisAngle.getX(), EPSILON);
         assertEquals(uy, axisAngle.getY(), EPSILON);
         assertEquals(uz, axisAngle.getZ(), EPSILON);
         assertEquals(angle, axisAngle.getAngle(), EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AxisAngle originalAxisAngle = new AxisAngle();
         EuclidCoreRandomTools.randomizeAxisAngle(random, 2.0 * Math.PI, originalAxisAngle);

         double qs = Math.cos(originalAxisAngle.getAngle() / 2.0);
         double qx = originalAxisAngle.getX() * Math.sin(originalAxisAngle.getAngle() / 2.0);
         double qy = originalAxisAngle.getY() * Math.sin(originalAxisAngle.getAngle() / 2.0);
         double qz = originalAxisAngle.getZ() * Math.sin(originalAxisAngle.getAngle() / 2.0);
         quaternion.setUnsafe(qx, qy, qz, qs);
         AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);

         EuclidCoreTestTools.assertAxisAngleEqualsSmart(originalAxisAngle, axisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }

      // Test with a quaternion that is not unitary
      double ux = EuclidCoreRandomTools.generateRandomDouble(random);
      double uy = EuclidCoreRandomTools.generateRandomDouble(random);
      double uz = EuclidCoreRandomTools.generateRandomDouble(random);

      double norm = Math.sqrt(ux * ux + uy * uy + uz * uz);
      ux /= norm;
      uy /= norm;
      uz /= norm;
      double angle = EuclidCoreRandomTools.generateRandomDouble(random, 2.0 * Math.PI);
      double scale = random.nextDouble();

      double qs = scale * Math.cos(angle / 2.0);
      double qx = scale * ux * Math.sin(angle / 2.0);
      double qy = scale * uy * Math.sin(angle / 2.0);
      double qz = scale * uz * Math.sin(angle / 2.0);
      quaternion.setUnsafe(qx, qy, qz, qs);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);

      if (axisAngle.getAngle() * angle < 0.0)
      {
         axisAngle.negate();
      }

      assertEquals(ux, axisAngle.getX(), EPSILON);
      assertEquals(uy, axisAngle.getY(), EPSILON);
      assertEquals(uz, axisAngle.getZ(), EPSILON);
      assertEquals(angle, axisAngle.getAngle(), EPSILON);
      EuclidCoreTestTools.assertAxisUnitary(axisAngle, EPSILON);

      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleIsSetToZero(axisAngle);

      quaternion.setUnsafe(0.0, 0.0, 0.0, Double.NaN);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      quaternion.setUnsafe(0.0, 0.0, Double.NaN, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      quaternion.setUnsafe(0.0, Double.NaN, 0.0, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      quaternion.setUnsafe(Double.NaN, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
   }

   @Test
   public void testRotationVectorToAxisAngle() throws Exception
   {
      Random random = new Random(2135L);
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      double minMaxAngleRange = 2.0 * Math.PI;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         EuclidCoreRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
         double rx = expectedAxisAngle.getX() * expectedAxisAngle.getAngle();
         double ry = expectedAxisAngle.getY() * expectedAxisAngle.getAngle();
         double rz = expectedAxisAngle.getZ() * expectedAxisAngle.getAngle();
         AxisAngleConversion.convertRotationVectorToAxisAngle(rx, ry, rz, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, 0.0, 0.0, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleIsSetToZero(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(Double.NaN, 0.0, 0.0, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, Double.NaN, 0.0, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, 0.0, Double.NaN, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      // Test with an actual vector
      for (int i = 0; i < 1000; i++)
      {
         Vector3D rotationVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D rotationVectorCopy = new Vector3D(rotationVector);
         AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), expectedAxisAngle);
         AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(rotationVector.equals(rotationVectorCopy));
      }
   }

   @Test
   public void testMatrixToAxisAngle() throws Exception
   {
      Random random = new Random(2135L);
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      double minMaxAngleRange = Math.PI;
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;
      RotationMatrix rotationMatrix = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         EuclidCoreRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
         double ux = expectedAxisAngle.getX();
         double uy = expectedAxisAngle.getY();
         double uz = expectedAxisAngle.getZ();
         double angle = expectedAxisAngle.getAngle();

         // The axis angle is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = cos(angle) + ux * ux * (1.0 - cos(angle));
         m11 = cos(angle) + uy * uy * (1.0 - cos(angle));
         m22 = cos(angle) + uz * uz * (1.0 - cos(angle));

         m01 = ux * uy * (1.0 - cos(angle)) - uz * sin(angle);
         m10 = ux * uy * (1.0 - cos(angle)) + uz * sin(angle);

         m20 = ux * uz * (1.0 - cos(angle)) - uy * sin(angle);
         m02 = ux * uz * (1.0 - cos(angle)) + uy * sin(angle);

         m12 = uy * uz * (1.0 - cos(angle)) - ux * sin(angle);
         m21 = uy * uz * (1.0 - cos(angle)) + ux * sin(angle);

         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         expectedAxisAngle.setAngle(Math.PI);
         Vector3D randomVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         randomVector.normalize();
         expectedAxisAngle.setX(randomVector.getX());
         expectedAxisAngle.setY(randomVector.getY());
         expectedAxisAngle.setZ(randomVector.getZ());
         double ux = expectedAxisAngle.getX();
         double uy = expectedAxisAngle.getY();
         double uz = expectedAxisAngle.getZ();
         double angle = expectedAxisAngle.getAngle();

         // The axis angle is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = cos(angle) + ux * ux * (1.0 - cos(angle));
         m11 = cos(angle) + uy * uy * (1.0 - cos(angle));
         m22 = cos(angle) + uz * uz * (1.0 - cos(angle));

         m01 = ux * uy * (1.0 - cos(angle)) - uz * sin(angle);
         m10 = ux * uy * (1.0 - cos(angle)) + uz * sin(angle);

         m20 = ux * uz * (1.0 - cos(angle)) - uy * sin(angle);
         m02 = ux * uz * (1.0 - cos(angle)) + uy * sin(angle);

         m12 = uy * uz * (1.0 - cos(angle)) - ux * sin(angle);
         m21 = uy * uz * (1.0 - cos(angle)) + ux * sin(angle);

         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      // Test edge cases
      // Zero rotation
      m00 = m11 = m22 = 1.0;
      m01 = m02 = m12 = 0.0;
      m10 = m20 = m21 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleIsSetToZero(actualAxisAngle);

      // Pi/2 around x
      m00 = 1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 0.0;
      m12 = -1.0;
      m20 = 0.0;
      m21 = 1.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(1.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

      // Pi around x
      m00 = 1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = -1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(1.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi/2 around y
      m00 = 0.0;
      m01 = 0.0;
      m02 = 1.0;
      m10 = 0.0;
      m11 = 1.0;
      m12 = 0.0;
      m20 = -1.0;
      m21 = 0.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

      // Pi around z
      m00 = -1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi/2 around z
      m00 = 0.0;
      m01 = -1.0;
      m02 = 0.0;
      m10 = 1.0;
      m11 = 0.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = 1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

      // Pi around z
      m00 = -1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = -1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = 1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around xy (as axis-angle: (x = sqrt(2)/2, y = sqrt(2)/2, z = 0, angle = Pi)
      double sqrt2Over2 = Math.sqrt(2.0) / 2.0;
      m00 = 0.0;
      m01 = 1.0;
      m02 = 0.0;
      m10 = 1.0;
      m11 = 0.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(sqrt2Over2, actualAxisAngle.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around xz (as axis-angle: (x = sqrt(2)/2, y = 0, z = sqrt(2)/2, angle = Pi)
      m00 = 0.0;
      m01 = 0.0;
      m02 = 1.0;
      m10 = 0.0;
      m11 = -1.0;
      m12 = 0.0;
      m20 = 1.0;
      m21 = 0.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(sqrt2Over2, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around yz (as axis-angle: (x = 0, y = sqrt(2)/2, z = sqrt(2)/2, angle = Pi)
      m00 = -1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 0.0;
      m12 = 1.0;
      m20 = 0.0;
      m21 = 1.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      rotationMatrix.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      // Test with a RotationScaleMatrix
      for (int i = 0; i < 1000; i++)
      {
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         RotationScaleMatrixReadOnly rotationScaleMatrixCopy = new RotationScaleMatrix(rotationScaleMatrix);
         m00 = rotationScaleMatrix.getRotationMatrix().getM00();
         m01 = rotationScaleMatrix.getRotationMatrix().getM01();
         m02 = rotationScaleMatrix.getRotationMatrix().getM02();
         m10 = rotationScaleMatrix.getRotationMatrix().getM10();
         m11 = rotationScaleMatrix.getRotationMatrix().getM11();
         m12 = rotationScaleMatrix.getRotationMatrix().getM12();
         m20 = rotationScaleMatrix.getRotationMatrix().getM20();
         m21 = rotationScaleMatrix.getRotationMatrix().getM21();
         m22 = rotationScaleMatrix.getRotationMatrix().getM22();
         AxisAngleConversion.convertMatrixToAxisAngle(rotationScaleMatrix, actualAxisAngle);
         AxisAngleConversion.convertMatrixToAxisAngle(rotationScaleMatrix.getRotationMatrix(), expectedAxisAngle);
         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
         // Assert the parameter does not get modified
         assertEquals(rotationScaleMatrix, rotationScaleMatrixCopy);
      }
   }

   @Test
   public void testYawPitchRollToAxisAngle() throws Exception
   {
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      RotationMatrix rotationMatrix = new RotationMatrix();
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
         {
            for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
            {
               double cYaw = Math.cos(yaw);
               double sYaw = Math.sin(yaw);
               double cPitch = Math.cos(pitch);
               double sPitch = Math.sin(pitch);
               double cRoll = Math.cos(roll);
               double sRoll = Math.sin(roll);

               m00 = cYaw * cPitch;
               m01 = cYaw * sPitch * sRoll - sYaw * cRoll;
               m02 = cYaw * sPitch * cRoll + sYaw * sRoll;
               m10 = sYaw * cPitch;
               m11 = sYaw * sPitch * sRoll + cYaw * cRoll;
               m12 = sYaw * sPitch * cRoll - cYaw * sRoll;
               m20 = -sPitch;
               m21 = cPitch * sRoll;
               m22 = cPitch * cRoll;

               rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
               AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, expectedAxisAngle);
               AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, actualAxisAngle);
               EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);

               EuclidCoreTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
            }
         }
      }
   }
}
