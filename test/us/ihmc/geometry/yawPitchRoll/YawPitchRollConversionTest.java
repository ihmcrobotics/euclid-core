package us.ihmc.geometry.yawPitchRoll;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngleConversion;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationMatrixConversion;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.tools.EuclidCoreRandomTools;
import us.ihmc.geometry.tools.EuclidCoreTestTools;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.QuaternionConversion;

public class YawPitchRollConversionTest
{
   private static final double EPSILON = 1.0e-12;
   private static final double MIN_PITCH_ANGLE = YawPitchRollConversion.MIN_PITCH_ANGLE + EPSILON;
   private static final double MAX_PITCH_ANGLE = YawPitchRollConversion.MAX_PITCH_ANGLE - EPSILON;

   @Test
   public void testMatrixToYawPitchRoll() throws Exception
   {
      Random random = new Random(923748L);
      RotationMatrix matrix = new RotationMatrix();
      double[] actualYawPitchRoll = new double[3];
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitch, roll, matrix);

               double actualYaw = YawPitchRollConversion.computeYawImpl(matrix.getM00(), matrix.getM10());
               assertEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitchImpl(matrix.getM20());
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRollImpl(matrix.getM21(), matrix.getM22());
               assertEquals(roll, actualRoll, EPSILON);

               actualYaw = YawPitchRollConversion.computeYaw(matrix);
               assertEquals(yaw, actualYaw, EPSILON);

               actualPitch = YawPitchRollConversion.computePitch(matrix);
               assertEquals(pitch, actualPitch, EPSILON);

               actualRoll = YawPitchRollConversion.computeRoll(matrix);
               assertEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualYawPitchRoll);
               assertEquals(yaw, actualYawPitchRoll[0], EPSILON);
               assertEquals(pitch, actualYawPitchRoll[1], EPSILON);
               assertEquals(roll, actualYawPitchRoll[2], EPSILON);

               YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualEulerAngles);
               assertEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               assertEquals(roll, actualEulerAngles.getX(), EPSILON);
            }

            // Make sure min and max pitch angles are tested
            double pitchMin = MIN_PITCH_ANGLE;
            RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitchMin, roll, matrix);

            double actualYaw = YawPitchRollConversion.computeYawImpl(matrix.getM00(), matrix.getM10());
            assertEquals(yaw, actualYaw, EPSILON);

            double actualPitch = YawPitchRollConversion.computePitchImpl(matrix.getM20());
            assertEquals(pitchMin, actualPitch, EPSILON);

            double actualRoll = YawPitchRollConversion.computeRollImpl(matrix.getM21(), matrix.getM22());
            assertEquals(roll, actualRoll, EPSILON);

            // Make sure min and max pitch angles are tested
            double pitchMax = MAX_PITCH_ANGLE;
            RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitchMax, roll, matrix);

            actualYaw = YawPitchRollConversion.computeYawImpl(matrix.getM00(), matrix.getM10());
            assertEquals(yaw, actualYaw, EPSILON);

            actualPitch = YawPitchRollConversion.computePitchImpl(matrix.getM20());
            assertEquals(pitchMax, actualPitch, EPSILON);

            actualRoll = YawPitchRollConversion.computeRollImpl(matrix.getM21(), matrix.getM22());
            assertEquals(roll, actualRoll, EPSILON);
         }
      }

      double actualYaw = YawPitchRollConversion.computeYawImpl(1.0, 0.0);
      assertTrue(actualYaw == 0.0);
      actualYaw = YawPitchRollConversion.computeYawImpl(Double.NaN, 0.0);
      assertTrue(Double.isNaN(actualYaw));
      actualYaw = YawPitchRollConversion.computeYawImpl(0.0, Double.NaN);
      assertTrue(Double.isNaN(actualYaw));

      double actualPitch = YawPitchRollConversion.computePitchImpl(0.0);
      assertTrue(actualPitch == 0.0);
      actualPitch = YawPitchRollConversion.computePitchImpl(Double.NaN);
      assertTrue(Double.isNaN(actualPitch));

      double actualRoll = YawPitchRollConversion.computeRollImpl(0.0, 1.0);
      assertTrue(actualRoll == 0.0);
      actualRoll = YawPitchRollConversion.computeRollImpl(Double.NaN, 0.0);
      assertTrue(Double.isNaN(actualRoll));
      actualRoll = YawPitchRollConversion.computeRollImpl(0.0, Double.NaN);
      assertTrue(Double.isNaN(actualRoll));

      // Assert that we can NaNs around the yawPitchRoll singularity
      actualPitch = YawPitchRollConversion.computePitchImpl(-Math.sin(YawPitchRollConversion.MAX_PITCH_ANGLE + EPSILON));
      assertTrue(Double.isNaN(actualPitch));
      actualPitch = YawPitchRollConversion.computePitchImpl(-Math.sin(YawPitchRollConversion.MIN_PITCH_ANGLE - EPSILON));
      assertTrue(Double.isNaN(actualPitch));

      for (int i = 0; i < 1000; i++)
      {
         double pitchOverMax = YawPitchRollConversion.MAX_PITCH_ANGLE + EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitchOverMax, roll, matrix);

         actualYaw = YawPitchRollConversion.computeYaw(matrix);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(matrix);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(matrix);
         assertTrue(Double.isNaN(actualRoll));

         YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));

         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         rotationScaleMatrix.setYawPitchRoll(yaw, pitchOverMax, roll);
         YawPitchRollConversion.convertMatrixToYawPitchRoll(rotationScaleMatrix, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         YawPitchRollConversion.convertMatrixToYawPitchRoll(rotationScaleMatrix, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }

      for (int i = 0; i < 1000; i++)
      {
         double pitchUnderMin = YawPitchRollConversion.MIN_PITCH_ANGLE - EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitchUnderMin, roll, matrix);

         actualYaw = YawPitchRollConversion.computeYaw(matrix);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(matrix);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(matrix);
         assertTrue(Double.isNaN(actualRoll));

         YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));

         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         rotationScaleMatrix.setYawPitchRoll(yaw, pitchUnderMin, roll);
         YawPitchRollConversion.convertMatrixToYawPitchRoll(rotationScaleMatrix, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         YawPitchRollConversion.convertMatrixToYawPitchRoll(rotationScaleMatrix, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }
   }

   @Test
   public void testQuaternionToYawPitchRoll() throws Exception
   {
      Quaternion quaternion = new Quaternion();
      double[] actualYawPitchRoll = new double[3];
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, quaternion);

               double qx = quaternion.getX();
               double qy = quaternion.getY();
               double qz = quaternion.getZ();
               double qs = quaternion.getS();

               double actualYaw = YawPitchRollConversion.computeYawFromQuaternionImpl(qx, qy, qz, qs);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitchFromQuaternionImpl(qx, qy, qz, qs);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRollFromQuaternionImpl(qx, qy, qz, qs);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               actualYaw = YawPitchRollConversion.computeYaw(quaternion);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               actualPitch = YawPitchRollConversion.computePitch(quaternion);
               assertEquals(pitch, actualPitch, EPSILON);

               actualRoll = YawPitchRollConversion.computeRoll(quaternion);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll[0], EPSILON);
               assertEquals(pitch, actualYawPitchRoll[1], EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll[2], EPSILON);

               YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
            }

            // Make sure min and max pitch angles are tested
            double pitchMin = MIN_PITCH_ANGLE;
            QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitchMin, roll, quaternion);

            double qx = quaternion.getX();
            double qy = quaternion.getY();
            double qz = quaternion.getZ();
            double qs = quaternion.getS();

            double actualPitch = YawPitchRollConversion.computePitchFromQuaternionImpl(qx, qy, qz, qs);
            assertEquals(pitchMin, actualPitch, EPSILON);

            double actualYaw = YawPitchRollConversion.computeYawFromQuaternionImpl(qx, qy, qz, qs);
            EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

            double actualRoll = YawPitchRollConversion.computeRollFromQuaternionImpl(qx, qy, qz, qs);
            EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

            // Make sure min and max pitch angles are tested
            double pitchMax = MAX_PITCH_ANGLE;
            QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitchMax, roll, quaternion);

            qx = quaternion.getX();
            qy = quaternion.getY();
            qz = quaternion.getZ();
            qs = quaternion.getS();

            actualPitch = YawPitchRollConversion.computePitchFromQuaternionImpl(qx, qy, qz, qs);
            assertEquals(pitchMax, actualPitch, EPSILON);

            actualYaw = YawPitchRollConversion.computeYawFromQuaternionImpl(qx, qy, qz, qs);
            EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

            actualRoll = YawPitchRollConversion.computeRollFromQuaternionImpl(qx, qy, qz, qs);
            EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);
         }
      }

      Random random = new Random(239478L);

      // Test with a non-unitary quaternion
      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, quaternion);
               double randomScale = EuclidCoreRandomTools.generateRandomDouble(random, 0.1, 10.0);
               quaternion.setUnsafe(quaternion.getX() * randomScale, quaternion.getY() * randomScale, quaternion.getZ() * randomScale,
                                    quaternion.getS() * randomScale);

               double actualYaw = YawPitchRollConversion.computeYaw(quaternion);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitch(quaternion);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRoll(quaternion);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll[0], EPSILON);
               assertEquals(pitch, actualYawPitchRoll[1], EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll[2], EPSILON);

               YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
            }
         }
      }

      // Test the checks on the norm
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);

      double actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(actualYaw == 0.0);

      double actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(actualPitch == 0.0);

      double actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(actualRoll == 0.0);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(actualYawPitchRoll[0] == 0.0);
      assertTrue(actualYawPitchRoll[1] == 0.0);
      assertTrue(actualYawPitchRoll[2] == 0.0);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualEulerAngles);

      // Test the checks on NaNs
      quaternion.setUnsafe(Double.NaN, 0.0, 0.0, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      quaternion.setUnsafe(0.0, Double.NaN, 0.0, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      quaternion.setUnsafe(0.0, 0.0, Double.NaN, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      quaternion.setUnsafe(0.0, 0.0, 0.0, Double.NaN);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      // Assert that we can NaNs around the yawPitchRoll singularity
      for (int i = 0; i < 1000; i++)
      {
         double pitchOverMax = YawPitchRollConversion.MAX_PITCH_ANGLE + EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitchOverMax, roll, quaternion);

         actualYaw = YawPitchRollConversion.computeYaw(quaternion);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(quaternion);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(quaternion);
         assertTrue(Double.isNaN(actualRoll));

         Arrays.fill(actualYawPitchRoll, 0.0);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         actualEulerAngles.set(0.0, 0.0, 0.0);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }

      for (int i = 0; i < 1000; i++)
      {
         double pitchUnderMin = YawPitchRollConversion.MIN_PITCH_ANGLE - EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitchUnderMin, roll, quaternion);

         actualYaw = YawPitchRollConversion.computeYaw(quaternion);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(quaternion);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(quaternion);
         assertTrue(Double.isNaN(actualRoll));

         Arrays.fill(actualYawPitchRoll, 0.0);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         actualEulerAngles.set(0.0, 0.0, 0.0);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }
   }

   @Test
   public void testAxisAngleToYawPitchRoll() throws Exception
   {
      AxisAngle axisAngle = new AxisAngle();
      double[] actualYawPitchRoll = new double[3];
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, axisAngle);

               double ux = axisAngle.getX();
               double uy = axisAngle.getY();
               double uz = axisAngle.getZ();
               double angle = axisAngle.getAngle();

               double actualYaw = YawPitchRollConversion.computeYawFromAxisAngleImpl(ux, uy, uz, angle);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitchFromAxisAngleImpl(ux, uy, uz, angle);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRollFromAxisAngleImpl(ux, uy, uz, angle);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               actualPitch = YawPitchRollConversion.computePitch(axisAngle);
               assertEquals(pitch, actualPitch, EPSILON);

               actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll[0], EPSILON);
               assertEquals(pitch, actualYawPitchRoll[1], EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll[2], EPSILON);

               YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
            }

            // Make sure min and max pitch angles are tested
            double pitchMin = MIN_PITCH_ANGLE;
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitchMin, roll, axisAngle);

            double ux = axisAngle.getX();
            double uy = axisAngle.getY();
            double uz = axisAngle.getZ();
            double angle = axisAngle.getAngle();

            double actualPitch = YawPitchRollConversion.computePitchFromAxisAngleImpl(ux, uy, uz, angle);
            assertEquals(pitchMin, actualPitch, EPSILON);

            double actualYaw = YawPitchRollConversion.computeYawFromAxisAngleImpl(ux, uy, uz, angle);
            EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

            double actualRoll = YawPitchRollConversion.computeRollFromAxisAngleImpl(ux, uy, uz, angle);
            EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

            // Make sure min and max pitch angles are tested
            double pitchMax = MAX_PITCH_ANGLE;
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitchMax, roll, axisAngle);

            ux = axisAngle.getX();
            uy = axisAngle.getY();
            uz = axisAngle.getZ();
            angle = axisAngle.getAngle();

            actualPitch = YawPitchRollConversion.computePitchFromAxisAngleImpl(ux, uy, uz, angle);
            assertEquals(pitchMax, actualPitch, EPSILON);

            actualYaw = YawPitchRollConversion.computeYawFromAxisAngleImpl(ux, uy, uz, angle);
            EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

            actualRoll = YawPitchRollConversion.computeRollFromAxisAngleImpl(ux, uy, uz, angle);
            EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);
         }
      }

      Random random = new Random(239478L);

      // Test with a non-unitary axes
      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, axisAngle);
               double randomScale = EuclidCoreRandomTools.generateRandomDouble(random, 0.1, 10.0);
               axisAngle.setX(axisAngle.getX() * randomScale);
               axisAngle.setY(axisAngle.getY() * randomScale);
               axisAngle.setZ(axisAngle.getZ() * randomScale);

               double actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitch(axisAngle);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll[0], EPSILON);
               assertEquals(pitch, actualYawPitchRoll[1], EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll[2], EPSILON);

               YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
            }
         }
      }

      // Test the checks on the norm
      axisAngle.set(0.0, 0.0, 0.0, 1.0);
      double actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(actualYaw == 0.0);

      double actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(actualPitch == 0.0);

      double actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(actualRoll == 0.0);

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(actualYawPitchRoll[0] == 0.0);
      assertTrue(actualYawPitchRoll[1] == 0.0);
      assertTrue(actualYawPitchRoll[2] == 0.0);

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualEulerAngles);

      // Test the checks on NaNs
      axisAngle.set(Double.NaN, 0.0, 0.0, 1.0);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      axisAngle.set(0.0, Double.NaN, 0.0, 1.0);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      axisAngle.set(0.0, 0.0, Double.NaN, 1.0);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      axisAngle.set(0.0, 0.0, 0.0, Double.NaN);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      // Assert that we can NaNs around the yawPitchRoll singularity
      for (int i = 0; i < 1000; i++)
      {
         double pitchOverMax = YawPitchRollConversion.MAX_PITCH_ANGLE + EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitchOverMax, roll, axisAngle);

         actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(axisAngle);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
         assertTrue(Double.isNaN(actualRoll));

         Arrays.fill(actualYawPitchRoll, 0.0);
         YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         actualEulerAngles.set(0.0, 0.0, 0.0);
         YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }

      for (int i = 0; i < 1000; i++)
      {
         double pitchUnderMin = YawPitchRollConversion.MIN_PITCH_ANGLE - EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitchUnderMin, roll, axisAngle);

         actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(axisAngle);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
         assertTrue(Double.isNaN(actualRoll));

         Arrays.fill(actualYawPitchRoll, 0.0);
         YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         actualEulerAngles.set(0.0, 0.0, 0.0);
         YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }
   }

   @Test
   public void testRotationVectorToYawPitchRoll() throws Exception
   {

      Vector3D rotationVector = new Vector3D();
      Vector3D rotationVectorCopy = new Vector3D();
      double[] actualYawPitchRoll = new double[3];
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               RotationVectorConversion.convertYawPitchRollToRotationVector(yaw, pitch, roll, rotationVector);
               rotationVectorCopy.set(rotationVector);

               double actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);
               assertTrue(rotationVector.equals(rotationVectorCopy));

               double actualPitch = YawPitchRollConversion.computePitch(rotationVector);
               assertEquals(pitch, actualPitch, EPSILON);
               assertTrue(rotationVector.equals(rotationVectorCopy));

               double actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);
               assertTrue(rotationVector.equals(rotationVectorCopy));

               YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll[0], EPSILON);
               assertEquals(pitch, actualYawPitchRoll[1], EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll[2], EPSILON);
               assertTrue(rotationVector.equals(rotationVectorCopy));

               YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
               assertTrue(rotationVector.equals(rotationVectorCopy));
            }
         }
      }

      Random random = new Random(239478L);

      // Test the checks on the norm
      rotationVector.setX(0.0);
      rotationVector.setY(0.0);
      rotationVector.setZ(0.0);

      double actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(actualYaw == 0.0);

      double actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(actualPitch == 0.0);

      double actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(actualRoll == 0.0);

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(actualYawPitchRoll[0] == 0.0);
      assertTrue(actualYawPitchRoll[1] == 0.0);
      assertTrue(actualYawPitchRoll[2] == 0.0);

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualEulerAngles);

      // Test the checks on NaNs
      rotationVector.set(Double.NaN, 0.0, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      rotationVector.set(0.0, Double.NaN, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      rotationVector.set(0.0, 0.0, Double.NaN);

      actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll[0]));
      assertTrue(Double.isNaN(actualYawPitchRoll[1]));
      assertTrue(Double.isNaN(actualYawPitchRoll[2]));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      for (int i = 0; i < 1000; i++)
      {
         double pitchOverMax = YawPitchRollConversion.MAX_PITCH_ANGLE + EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         RotationVectorConversion.convertYawPitchRollToRotationVector(yaw, pitchOverMax, roll, rotationVector);

         actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(rotationVector);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
         assertTrue(Double.isNaN(actualRoll));

         Arrays.fill(actualYawPitchRoll, 0.0);
         YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         actualEulerAngles.set(0.0, 0.0, 0.0);
         YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }

      for (int i = 0; i < 1000; i++)
      {
         double pitchUnderMin = YawPitchRollConversion.MIN_PITCH_ANGLE - EPSILON;
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         RotationVectorConversion.convertYawPitchRollToRotationVector(yaw, pitchUnderMin, roll, rotationVector);

         actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
         assertTrue(Double.isNaN(actualYaw));

         actualPitch = YawPitchRollConversion.computePitch(rotationVector);
         assertTrue(Double.isNaN(actualPitch));

         actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
         assertTrue(Double.isNaN(actualRoll));

         Arrays.fill(actualYawPitchRoll, 0.0);
         YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
         assertTrue(Double.isNaN(actualYawPitchRoll[0]));
         assertTrue(Double.isNaN(actualYawPitchRoll[1]));
         assertTrue(Double.isNaN(actualYawPitchRoll[2]));

         actualEulerAngles.set(0.0, 0.0, 0.0);
         YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
         assertTrue(Double.isNaN(actualEulerAngles.getX()));
         assertTrue(Double.isNaN(actualEulerAngles.getY()));
         assertTrue(Double.isNaN(actualEulerAngles.getZ()));
      }
   }
}
