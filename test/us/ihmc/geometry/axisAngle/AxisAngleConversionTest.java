package us.ihmc.geometry.axisAngle;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
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

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double ux = GeometryBasicsRandomTools.generateRandomDouble(random);
         double uy = GeometryBasicsRandomTools.generateRandomDouble(random);
         double uz = GeometryBasicsRandomTools.generateRandomDouble(random);

         double norm = Math.sqrt(ux * ux + uy * uy + uz * uz);
         ux /= norm;
         uy /= norm;
         uz /= norm;
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 2.0 * Math.PI);

         double qs = Math.cos(angle / 2.0);
         double qx = ux * Math.sin(angle / 2.0);
         double qy = uy * Math.sin(angle / 2.0);
         double qz = uz * Math.sin(angle / 2.0);
         AxisAngleConversion.convertQuaternionToAxisAngle(qx, qy, qz, qs, axisAngle);

         if (axisAngle.getAngle() * angle < 0.0)
         {
            axisAngle.negate();
         }

         assertEquals(ux, axisAngle.getX(), EPSILON);
         assertEquals(uy, axisAngle.getY(), EPSILON);
         assertEquals(uz, axisAngle.getZ(), EPSILON);
         assertEquals(angle, axisAngle.getAngle(), EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AxisAngle originalAxisAngle = new AxisAngle();
         GeometryBasicsRandomTools.randomizeAxisAngle(random, 2.0 * Math.PI, originalAxisAngle);

         double qs = Math.cos(originalAxisAngle.getAngle() / 2.0);
         double qx = originalAxisAngle.getX() * Math.sin(originalAxisAngle.getAngle() / 2.0);
         double qy = originalAxisAngle.getY() * Math.sin(originalAxisAngle.getAngle() / 2.0);
         double qz = originalAxisAngle.getZ() * Math.sin(originalAxisAngle.getAngle() / 2.0);
         AxisAngleConversion.convertQuaternionToAxisAngle(qx, qy, qz, qs, axisAngle);

         GeometryBasicsTestTools.assertAxisAngleEqualsSmart(originalAxisAngle, axisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }

      // Test with a quaternion that is not unitary
      double ux = GeometryBasicsRandomTools.generateRandomDouble(random);
      double uy = GeometryBasicsRandomTools.generateRandomDouble(random);
      double uz = GeometryBasicsRandomTools.generateRandomDouble(random);

      double norm = Math.sqrt(ux * ux + uy * uy + uz * uz);
      ux /= norm;
      uy /= norm;
      uz /= norm;
      double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 2.0 * Math.PI);
      double scale = random.nextDouble();

      double qs = scale * Math.cos(angle / 2.0);
      double qx = scale * ux * Math.sin(angle / 2.0);
      double qy = scale * uy * Math.sin(angle / 2.0);
      double qz = scale * uz * Math.sin(angle / 2.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(qx, qy, qz, qs, axisAngle);

      if (axisAngle.getAngle() * angle < 0.0)
      {
         axisAngle.negate();
      }

      assertEquals(ux, axisAngle.getX(), EPSILON);
      assertEquals(uy, axisAngle.getY(), EPSILON);
      assertEquals(uz, axisAngle.getZ(), EPSILON);
      assertEquals(angle, axisAngle.getAngle(), EPSILON);
      GeometryBasicsTestTools.assertAxisUnitary(axisAngle, EPSILON);

      AxisAngleConversion.convertQuaternionToAxisAngle(0.0, 0.0, 0.0, 0.0, axisAngle);
      GeometryBasicsTestTools.assertAxisAngleIsSetToZero(axisAngle);

      AxisAngleConversion.convertQuaternionToAxisAngle(0.0, 0.0, 0.0, Double.NaN, axisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      AxisAngleConversion.convertQuaternionToAxisAngle(0.0, 0.0, Double.NaN, 0.0, axisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      AxisAngleConversion.convertQuaternionToAxisAngle(0.0, Double.NaN, 0.0, 0.0, axisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      AxisAngleConversion.convertQuaternionToAxisAngle(Double.NaN, 0.0, 0.0, 0.0, axisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      // Test with an actual Quaternion
      AxisAngle expectedAxisAngle = new AxisAngle();
      for (int i = 0; i < 1000; i++)
      {
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
         AxisAngleConversion.convertQuaternionToAxisAngle(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), expectedAxisAngle);
         assertTrue(expectedAxisAngle.getX() == axisAngle.getX());
         assertTrue(expectedAxisAngle.getY() == axisAngle.getY());
         assertTrue(expectedAxisAngle.getZ() == axisAngle.getZ());
         assertTrue(expectedAxisAngle.getAngle() == axisAngle.getAngle());
         GeometryBasicsTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }
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
         GeometryBasicsRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
         double rx = expectedAxisAngle.getX() * expectedAxisAngle.getAngle();
         double ry = expectedAxisAngle.getY() * expectedAxisAngle.getAngle();
         double rz = expectedAxisAngle.getZ() * expectedAxisAngle.getAngle();
         AxisAngleConversion.convertRotationVectorToAxisAngle(rx, ry, rz, actualAxisAngle);

         GeometryBasicsTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleIsSetToZero(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(Double.NaN, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, Double.NaN, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, 0.0, Double.NaN, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      // Test with an actual vector
      for (int i = 0; i < 1000; i++)
      {
         Vector3D rotationVector = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Vector3D rotationVectorCopy = new Vector3D(rotationVector);
         AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), expectedAxisAngle);
         AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, actualAxisAngle);

         GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
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

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
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

         AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);

         GeometryBasicsTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         expectedAxisAngle.setAngle(Math.PI);
         Vector3D randomVector = GeometryBasicsRandomTools.generateRandomVector3D(random);
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

         AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);

         GeometryBasicsTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      // Test edge cases
      // Zero rotation
      m00 = m11 = m22 = 1.0;
      m01 = m02 = m12 = 0.0;
      m10 = m20 = m21 = 0.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleIsSetToZero(actualAxisAngle);

      // Pi/2 around x
      m00 = 1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = 0.0; m12 = -1.0;
      m20 = 0.0; m21 = 1.0; m22 = 0.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(1.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

      // Pi around x
      m00 = 1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = -1.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = -1.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(1.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi/2 around y
      m00 = 0.0; m01 = 0.0; m02 = 1.0;
      m10 = 0.0; m11 = 1.0; m12 = 0.0;
      m20 = -1.0; m21 = 0.0; m22 = 0.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

      // Pi around z
      m00 = -1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = 1.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = -1.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi/2 around z
      m00 = 0.0; m01 = -1.0; m02 = 0.0;
      m10 = 1.0; m11 = 0.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = 1.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

      // Pi around z
      m00 = -1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = -1.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = 1.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around xy (as axis-angle: (x = sqrt(2)/2, y = sqrt(2)/2, z = 0, angle = Pi)
      double sqrt2Over2 = Math.sqrt(2.0) / 2.0;
      m00 = 0.0; m01 = 1.0; m02 = 0.0;
      m10 = 1.0; m11 = 0.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = -1.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(sqrt2Over2, actualAxisAngle.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around xz (as axis-angle: (x = sqrt(2)/2, y = 0, z = sqrt(2)/2, angle = Pi)
      m00 = 0.0; m01 = 0.0; m02 = 1.0;
      m10 = 0.0; m11 = -1.0; m12 = 0.0;
      m20 = 1.0; m21 = 0.0; m22 = 0.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(sqrt2Over2, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around yz (as axis-angle: (x = 0, y = sqrt(2)/2, z = sqrt(2)/2, angle = Pi)
      m00 = -1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = 0.0; m12 = 1.0;
      m20 = 0.0; m21 = 1.0; m22 = 0.0;
      AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      AxisAngleConversion.convertMatrixToAxisAngleImpl(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      AxisAngleConversion.convertMatrixToAxisAngleImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, actualAxisAngle);
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      // Test with an actual matrix
      for (int i = 0; i < 1000; i++)
      {
         RotationMatrix rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix rotationMatrixCopy = new RotationMatrix(rotationMatrix);
         m00 = rotationMatrix.getM00();
         m01 = rotationMatrix.getM01();
         m02 = rotationMatrix.getM02();
         m10 = rotationMatrix.getM10();
         m11 = rotationMatrix.getM11();
         m12 = rotationMatrix.getM12();
         m20 = rotationMatrix.getM20();
         m21 = rotationMatrix.getM21();
         m22 = rotationMatrix.getM22();
         AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
         AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedAxisAngle);
         GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
         // Assert the parameter does not get modified
         assertTrue(rotationMatrix.equals(rotationMatrixCopy));
      }

      // Test with a RotationScaleMatrix
      for (int i = 0; i < 1000; i++)
      {
         RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         RotationScaleMatrixReadOnly<?> rotationScaleMatrixCopy = new RotationScaleMatrix(rotationScaleMatrix);
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
         AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedAxisAngle);
         GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
         // Assert the parameter does not get modified
         assertEquals(rotationScaleMatrix, rotationScaleMatrixCopy);
      }
   }

   @Test
   public void testYawPitchRollToAxisAngle() throws Exception
   {
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
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

               AxisAngleConversion.convertMatrixToAxisAngleImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedAxisAngle);
               AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, actualAxisAngle);
               GeometryBasicsTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);

               GeometryBasicsTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
            }
         }
      }
   }
}
