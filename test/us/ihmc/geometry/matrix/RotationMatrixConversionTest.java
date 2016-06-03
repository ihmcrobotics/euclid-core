package us.ihmc.geometry.matrix;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngleConversion;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple.Point;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.QuaternionConversion;

public class RotationMatrixConversionTest
{
   private static final double EPSILON = 1.0e-12;
   public static final int NUMBER_OF_ITERATIONS = 2000;

   @Test
   public void testYawPitchRollToMatrix() throws Exception
   {
      Random random = new Random(230487L);
      RotationMatrix expectedMatrix = new RotationMatrix();
      RotationMatrix actualMatrix = new RotationMatrix();

      RotationMatrix yawMatrix = new RotationMatrix();
      RotationMatrix pitchMatrix = new RotationMatrix();
      RotationMatrix rollMatrix = new RotationMatrix();

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         yawMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrixConversion.computeYawMatrix(yaw, yawMatrix);
         assertTrue(yawMatrix.getM22() == 1.0);
         assertTrue(yawMatrix.getM12() == 0.0);
         assertTrue(yawMatrix.getM02() == 0.0);
         assertTrue(yawMatrix.getM21() == 0.0);
         assertTrue(yawMatrix.getM20() == 0.0);
         assertEquals(yawMatrix.getM00(), Math.cos(yaw), EPSILON);
         assertEquals(yawMatrix.getM11(), Math.cos(yaw), EPSILON);
         assertEquals(yawMatrix.getM10(), Math.sin(yaw), EPSILON);
         assertEquals(yawMatrix.getM01(), -Math.sin(yaw), EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(yawMatrix, EPSILON);

         for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
         {
            pitchMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            RotationMatrixConversion.computePitchMatrix(pitch, pitchMatrix);
            assertTrue(pitchMatrix.getM11() == 1.0);
            assertTrue(pitchMatrix.getM01() == 0.0);
            assertTrue(pitchMatrix.getM10() == 0.0);
            assertTrue(pitchMatrix.getM21() == 0.0);
            assertTrue(pitchMatrix.getM12() == 0.0);
            assertEquals(pitchMatrix.getM00(), Math.cos(pitch), EPSILON);
            assertEquals(pitchMatrix.getM22(), Math.cos(pitch), EPSILON);
            assertEquals(pitchMatrix.getM20(), -Math.sin(pitch), EPSILON);
            assertEquals(pitchMatrix.getM02(), Math.sin(pitch), EPSILON);
            GeometryBasicsTestTools.assertRotationMatrix(pitchMatrix, EPSILON);

            for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
            {
               rollMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
               RotationMatrixConversion.computeRollMatrix(roll, rollMatrix);
               assertTrue(rollMatrix.getM00() == 1.0);
               assertTrue(rollMatrix.getM10() == 0.0);
               assertTrue(rollMatrix.getM20() == 0.0);
               assertTrue(rollMatrix.getM01() == 0.0);
               assertTrue(rollMatrix.getM02() == 0.0);
               assertEquals(rollMatrix.getM11(), Math.cos(roll), EPSILON);
               assertEquals(rollMatrix.getM22(), Math.cos(roll), EPSILON);
               assertEquals(rollMatrix.getM12(), -Math.sin(roll), EPSILON);
               assertEquals(rollMatrix.getM21(), Math.sin(roll), EPSILON);
               GeometryBasicsTestTools.assertRotationMatrix(rollMatrix, EPSILON);

               RotationMatrixTools.multiply(yawMatrix, pitchMatrix, expectedMatrix);
               RotationMatrixTools.multiply(expectedMatrix, rollMatrix, expectedMatrix);
               GeometryBasicsTestTools.assertRotationMatrix(expectedMatrix, EPSILON);

               RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitch, roll, actualMatrix);
               GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
               GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
            }
         }
      }

      RotationMatrixConversion.computeYawMatrix(0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.computeYawMatrix(0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.computePitchMatrix(0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.computeRollMatrix(0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.convertYawPitchRollToMatrix(0.0, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);
   }

   @Test
   public void testAxisAngleToMatrix() throws Exception
   {
      Random random = new Random(489651L);
      double minMaxAngleRange = Math.PI;
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      RotationMatrix actualMatrix = new RotationMatrix();
      RotationMatrix expectedMatrix = new RotationMatrix();

      for (int i = 0; i < RotationMatrixConversionTest.NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
         double ux = expectedAxisAngle.getX();
         double uy = expectedAxisAngle.getY();
         double uz = expectedAxisAngle.getZ();
         double angle = expectedAxisAngle.getAngle();
         RotationMatrixConversion.convertAxisAngleToMatrixImpl(ux, uy, uz, angle, actualMatrix);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         // Here we assume that the axis angle conversion is already well tested
         AxisAngleConversion.convertMatrixToAxisAngle(actualMatrix, actualAxisAngle);
         GeometryBasicsTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, EPSILON);
      }

      RotationMatrixConversion.convertAxisAngleToMatrixImpl(0.0, 0.0, 0.0, 1.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.convertAxisAngleToMatrixImpl(Double.NaN, 0.0, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertAxisAngleToMatrixImpl(0.0, Double.NaN, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertAxisAngleToMatrixImpl(0.0, 0.0, Double.NaN, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertAxisAngleToMatrixImpl(0.0, 0.0, 0.0, Double.NaN, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      // Test with an actual axis angle
      for (int i = 0; i < 1000; i++)
      {
         AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random, minMaxAngleRange);
         AxisAngle axisAngleCopy = new AxisAngle(axisAngle);

         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         RotationMatrixConversion.convertAxisAngleToMatrixImpl(ux, uy, uz, angle, expectedMatrix);
         RotationMatrixConversion.convertAxisAngleToMatrix(axisAngle, actualMatrix);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(axisAngle.equals(axisAngleCopy));
      }

      for (double angle = -Math.PI; angle <= Math.PI; angle += 0.01 * Math.PI)
      {
         RotationMatrixConversion.computeRollMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertAxisAngleToMatrixImpl(1.5 * random.nextDouble(), 0.0, 0.0, angle, actualMatrix);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);

         RotationMatrixConversion.computePitchMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertAxisAngleToMatrixImpl(0.0, 1.5 * random.nextDouble(), 0.0, angle, actualMatrix);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);

         RotationMatrixConversion.computeYawMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertAxisAngleToMatrixImpl(0.0, 0.0, 1.5 * random.nextDouble(), angle, actualMatrix);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
      }
   }

   @Test
   public void testQuaternionToMatrix() throws Exception
   {
      Random random = new Random(489651L);
      double minMaxAngleRange = Math.PI;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();
      RotationMatrix actualMatrix = new RotationMatrix();
      RotationMatrix expectedMatrix = new RotationMatrix();

      for (int i = 0; i < RotationMatrixConversionTest.NUMBER_OF_ITERATIONS; i++)
      {
         expectedQuaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, minMaxAngleRange);
         double qx = expectedQuaternion.getX();
         double qy = expectedQuaternion.getY();
         double qz = expectedQuaternion.getZ();
         double qs = expectedQuaternion.getS();
         RotationMatrixConversion.convertQuaternionToMatrixImpl(qx, qy, qz, qs, actualMatrix);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         // Assuming the quaternion conversion is well tested
         QuaternionConversion.convertMatrixToQuaternion(actualMatrix, actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPSILON);
      }

      RotationMatrixConversion.convertQuaternionToMatrixImpl(0.0, 0.0, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.convertQuaternionToMatrixImpl(Double.NaN, 0.0, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertQuaternionToMatrixImpl(0.0, Double.NaN, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertQuaternionToMatrixImpl(0.0, 0.0, Double.NaN, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertQuaternionToMatrixImpl(0.0, 0.0, 0.0, Double.NaN, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      for (double angle = -2.0 * Math.PI; angle <= 2.0 * Math.PI; angle += 0.01 * Math.PI)
      {
         double scale = 1.5 * random.nextDouble();
         RotationMatrixConversion.computeRollMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertQuaternionToMatrixImpl(scale * sin(angle / 2.0), 0.0, 0.0, scale * cos(angle / 2.0), actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);

         RotationMatrixConversion.computePitchMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertQuaternionToMatrixImpl(0.0, scale * sin(angle / 2.0), 0.0, scale * cos(angle / 2.0), actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);

         RotationMatrixConversion.computeYawMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertQuaternionToMatrixImpl(0.0, 0.0, scale * sin(angle / 2.0), scale * cos(angle / 2.0), actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
      }

      for (int i = 0; i < 1000; i++)
      {
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random, minMaxAngleRange);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         RotationMatrixConversion.convertQuaternionToMatrixImpl(qx, qy, qz, qs, expectedMatrix);
         RotationMatrixConversion.convertQuaternionToMatrix(quaternion, actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
      }
   }

   @Test
   public void testRotationVectorToMatrix() throws Exception
   {
      Random random = new Random(126515L);
      double minMaxAngleRange = Math.PI;
      RotationMatrix expectedMatrix = new RotationMatrix();
      RotationMatrix actualMatrix = new RotationMatrix();

      for (int i = 0; i < RotationMatrixConversionTest.NUMBER_OF_ITERATIONS; i++)
      {
         AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random, minMaxAngleRange);
         double rx = axisAngle.getX() * axisAngle.getAngle();
         double ry = axisAngle.getY() * axisAngle.getAngle();
         double rz = axisAngle.getZ() * axisAngle.getAngle();
         RotationMatrixConversion.convertAxisAngleToMatrix(axisAngle, expectedMatrix);
         RotationMatrixConversion.convertRotationVectorToMatrixImpl(rx, ry, rz, actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
      }

      RotationMatrixConversion.convertRotationVectorToMatrixImpl(0.0, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertIdentity(actualMatrix, EPSILON);

      RotationMatrixConversion.convertRotationVectorToMatrixImpl(Double.NaN, 0.0, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertRotationVectorToMatrixImpl(0.0, Double.NaN, 0.0, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      RotationMatrixConversion.convertRotationVectorToMatrixImpl(0.0, 0.0, Double.NaN, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(actualMatrix);

      for (double angle = -Math.PI; angle <= Math.PI; angle += 0.01 * Math.PI)
      {
         RotationMatrixConversion.computeRollMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertRotationVectorToMatrixImpl(angle, 0.0, 0.0, actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);

         RotationMatrixConversion.computePitchMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertRotationVectorToMatrixImpl(0.0, angle, 0.0, actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);

         RotationMatrixConversion.computeYawMatrix(angle, expectedMatrix);
         RotationMatrixConversion.convertRotationVectorToMatrixImpl(0.0, 0.0, angle, actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
      }

      // Test with an actual vector
      for (int i = 0; i < 1000; i++)
      {
         Vector rotationVector = GeometryBasicsRandomTools.generateRandomVector(random, new Point(minMaxAngleRange, minMaxAngleRange, minMaxAngleRange));
         Vector rotationVectorCopy = new Vector(rotationVector);
         double rx = rotationVector.getX();
         double ry = rotationVector.getY();
         double rz = rotationVector.getZ();
         RotationMatrixConversion.convertRotationVectorToMatrixImpl(rx, ry, rz, expectedMatrix);
         RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, actualMatrix);
         GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPSILON);
         GeometryBasicsTestTools.assertRotationMatrix(actualMatrix, EPSILON);
         assertTrue(rotationVector.equals(rotationVectorCopy));
      }
   }
}
