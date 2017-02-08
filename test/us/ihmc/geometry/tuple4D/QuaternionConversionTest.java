package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Vector3D;

public class QuaternionConversionTest
{
   private static final double EPSILON = 1.0e-12;
   public static final int NUMBER_OF_ITERATIONS = 2000;

   @Test
   public void testAxisAngleToQuaternion() throws Exception
   {
      Random random = new Random(484514L);
      double minMaxAngleRange = 2.0 * Math.PI;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random, minMaxAngleRange);
         double angle = axisAngle.getAngle();
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         // As the axis-angle is sane, there is no edge case making the conversion straightforward.
         double qs = Math.cos(angle / 2.0);
         double qx = ux * Math.sin(angle / 2.0);
         double qy = uy * Math.sin(angle / 2.0);
         double qz = uz * Math.sin(angle / 2.0);
         expectedQuaternion.setUnsafe(qx, qy, qz, qs);

         QuaternionConversion.convertAxisAngleToQuaternionImpl(ux, uy, uz, angle, actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      // Test with an axis-angle that has a non unnitary axis.
      double scale = random.nextDouble();
      AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random, minMaxAngleRange);
      double angle = axisAngle.getAngle();
      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      QuaternionConversion.convertAxisAngleToQuaternionImpl(ux, uy, uz, angle, expectedQuaternion);
      QuaternionConversion.convertAxisAngleToQuaternionImpl(scale * ux, scale * uy, scale * uz, angle, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
      GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);

      QuaternionConversion.convertAxisAngleToQuaternionImpl(0.0, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternionImpl(Double.NaN, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternionImpl(0.0, Double.NaN, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternionImpl(0.0, 0.0, Double.NaN, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternionImpl(0.0, 0.0, 0.0, Double.NaN, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      // Test with an actual quaternion
      for (int i = 0; i < 100; i++)
      {
         axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random, minMaxAngleRange);
         AxisAngle axisAngleCopy = new AxisAngle(axisAngle);
         angle = axisAngle.getAngle();
         ux = axisAngle.getX();
         uy = axisAngle.getY();
         uz = axisAngle.getZ();
         QuaternionConversion.convertAxisAngleToQuaternionImpl(ux, uy, uz, angle, expectedQuaternion);
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(axisAngle.equals(axisAngleCopy));
      }
   }

   @Test
   public void testMatrixToQuaternion() throws Exception
   {
      Random random = new Random(2135L);
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();
      double minMaxAngleRange = Math.PI;
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         expectedQuaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, minMaxAngleRange);
         double qx = expectedQuaternion.getX();
         double qy = expectedQuaternion.getY();
         double qz = expectedQuaternion.getZ();
         double qs = expectedQuaternion.getS();

         // The quaternion is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = 1.0 - 2.0 * (qy * qy + qz * qz);
         m11 = 1.0 - 2.0 * (qx * qx + qz * qz);
         m22 = 1.0 - 2.0 * (qx * qx + qy * qy);

         m01 = 2.0 * (qx * qy - qz * qs);
         m10 = 2.0 * (qx * qy + qz * qs);

         m20 = 2.0 * (qx * qz - qy * qs);
         m02 = 2.0 * (qx * qz + qy * qs);

         m12 = 2.0 * (qy * qz - qx * qs);
         m21 = 2.0 * (qy * qz + qx * qs);

         QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);

         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D randomVector = GeometryBasicsRandomTools.generateRandomVector3D(random);
         randomVector.normalize();
         expectedQuaternion.setUnsafe(randomVector.getX(), randomVector.getY(), randomVector.getZ(), 0.0); // rotation angle of Pi
         double qx = expectedQuaternion.getX();
         double qy = expectedQuaternion.getY();
         double qz = expectedQuaternion.getZ();
         double qs = expectedQuaternion.getS();

         // The quaternion is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = 1.0 - 2.0 * (qy * qy + qz * qz);
         m11 = 1.0 - 2.0 * (qx * qx + qz * qz);
         m22 = 1.0 - 2.0 * (qx * qx + qy * qy);

         m01 = 2.0 * (qx * qy - qz * qs);
         m10 = 2.0 * (qx * qy + qz * qs);

         m20 = 2.0 * (qx * qz - qy * qs);
         m02 = 2.0 * (qx * qz + qy * qs);

         m12 = 2.0 * (qy * qz - qx * qs);
         m21 = 2.0 * (qy * qz + qx * qs);

         QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      double sqrt2Over2 = Math.sqrt(2.0) / 2.0;
      // Test edge cases
      // Zero rotation
      m00 = m11 = m22 = 1.0;
      m01 = m02 = m12 = 0.0;
      m10 = m20 = m21 = 0.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      // Pi/2 around x
      m00 = 1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = 0.0; m12 = -1.0;
      m20 = 0.0; m21 = 1.0; m22 = 0.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(sqrt2Over2, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getS(), EPSILON);

      // Pi around x
      m00 = 1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = -1.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = -1.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(1.0, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi/2 around y
      m00 = 0.0; m01 = 0.0; m02 = 1.0;
      m10 = 0.0; m11 = 1.0; m12 = 0.0;
      m20 = -1.0; m21 = 0.0; m22 = 0.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getS(), EPSILON);

      // Pi around z
      m00 = -1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = 1.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = -1.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(1.0, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi/2 around z
      m00 = 0.0; m01 = -1.0; m02 = 0.0;
      m10 = 1.0; m11 = 0.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = 1.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getZ(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getS(), EPSILON);

      // Pi around z
      m00 = -1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = -1.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = 1.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(1.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi around xy (as axis-angle: (x = sqrt(2)/2, y = sqrt(2)/2, z = 0, angle = Pi)
      m00 = 0.0; m01 = 1.0; m02 = 0.0;
      m10 = 1.0; m11 = 0.0; m12 = 0.0;
      m20 = 0.0; m21 = 0.0; m22 = -1.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(sqrt2Over2, actualQuaternion.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi around xz (as axis-angle: (x = sqrt(2)/2, y = 0, z = sqrt(2)/2, angle = Pi)
      m00 = 0.0; m01 = 0.0; m02 = 1.0;
      m10 = 0.0; m11 = -1.0; m12 = 0.0;
      m20 = 1.0; m21 = 0.0; m22 = 0.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(sqrt2Over2, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi around yz (as axis-angle: (x = 0, y = sqrt(2)/2, z = sqrt(2)/2, angle = Pi)
      m00 = -1.0; m01 = 0.0; m02 = 0.0;
      m10 = 0.0; m11 = 0.0; m12 = 1.0;
      m20 = 0.0; m21 = 1.0; m22 = 0.0;
      QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      QuaternionConversion.convertMatrixToQuaternionImpl(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
      QuaternionConversion.convertMatrixToQuaternionImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

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
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
         QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         // Assert the parameter does not get modified
         assertTrue(rotationMatrix.equals(rotationMatrixCopy));
      }

      // Test with a RotationScaleMatrix
      for (int i = 0; i < 1000; i++)
      {
         RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         RotationScaleMatrix rotationScaleMatrixCopy = new RotationScaleMatrix(rotationScaleMatrix);
         m00 = rotationScaleMatrix.getRotationMatrix().getM00();
         m01 = rotationScaleMatrix.getRotationMatrix().getM01();
         m02 = rotationScaleMatrix.getRotationMatrix().getM02();
         m10 = rotationScaleMatrix.getRotationMatrix().getM10();
         m11 = rotationScaleMatrix.getRotationMatrix().getM11();
         m12 = rotationScaleMatrix.getRotationMatrix().getM12();
         m20 = rotationScaleMatrix.getRotationMatrix().getM20();
         m21 = rotationScaleMatrix.getRotationMatrix().getM21();
         m22 = rotationScaleMatrix.getRotationMatrix().getM22();
         QuaternionConversion.convertMatrixToQuaternion(rotationScaleMatrix, actualQuaternion);
         QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
         // Assert the parameter does not get modified
         assertTrue(rotationScaleMatrix.equals(rotationScaleMatrixCopy));
      }
   }

   @Test
   public void testYawPitchRollToQuaternion() throws Exception
   {
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();

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

               QuaternionConversion.convertMatrixToQuaternionImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedQuaternion);
               QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, actualQuaternion);
               GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPSILON);
               GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
            }
         }
      }

      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      QuaternionConversion.convertYawPitchRollToQuaternion(Double.NaN, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, Double.NaN, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, 0.0, Double.NaN, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);
   }

   @Test
   public void testRotationVectorToQuaternion() throws Exception
   {
      Random random = new Random(32047230L);
      double minMaxAngleRange = 2.0 * Math.PI;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random, minMaxAngleRange);
         double rx = axisAngle.getX() * axisAngle.getAngle();
         double ry = axisAngle.getY() * axisAngle.getAngle();
         double rz = axisAngle.getZ() * axisAngle.getAngle();
         // The axisangle->quaternion conversion is safe here as it is tested separately in testAxisAngleToQuaternion().
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, expectedQuaternion);
         QuaternionConversion.convertRotationVectorToQuaternionImpl(rx, ry, rz, actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      QuaternionConversion.convertRotationVectorToQuaternionImpl(0.0, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      QuaternionConversion.convertRotationVectorToQuaternionImpl(Double.NaN, 0.0, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertRotationVectorToQuaternionImpl(0.0, Double.NaN, 0.0, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertRotationVectorToQuaternionImpl(0.0, 0.0, Double.NaN, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(actualQuaternion);

      // Test with an actual vector
      Vector3D rotationVector = new Vector3D();
      Vector3D rotationVectorCopy = new Vector3D();

      for (int i = 0; i < 1000; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple3D(random, new Point3D(minMaxAngleRange, minMaxAngleRange, minMaxAngleRange), rotationVector);
         rotationVectorCopy.set(rotationVector);

         double rx = rotationVector.getX();
         double ry = rotationVector.getY();
         double rz = rotationVector.getZ();
         QuaternionConversion.convertRotationVectorToQuaternionImpl(rx, ry, rz, expectedQuaternion);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         GeometryBasicsTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(rotationVector.equals(rotationVectorCopy));
      }
   }
}
