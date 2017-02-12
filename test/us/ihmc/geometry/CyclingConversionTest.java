package us.ihmc.geometry;

import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngleConversion;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationMatrixConversion;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.QuaternionConversion;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public class CyclingConversionTest
{
   private static final double EPSILON = 1.0e-12;
   /** It seems that going through yaw-pitch-roll reduces the accuracy */
   private static final double EPSILON_WITH_YPR = 1.0e-11;
   private static final boolean DEBUG = false;

   @Test
   public void testCyclesIgnoringYawPitchRollWhenNaN() throws Exception
   {
      Random random = new Random(165416L);
      AllRotations[] rotationsToGoThrough = AllRotations.values();

      performConversionCycles(random, rotationsToGoThrough, 1000, 1000, EPSILON_WITH_YPR);
   }

   @Test
   public void testCyclesAllExceptYawPitchRoll() throws Exception
   {
      Random random = new Random(165416L);
      AllRotations[] rotationsToGoThrough = {AllRotations.QUATERNION, AllRotations.AXISANGLE, AllRotations.MATRIX, AllRotations.VECTOR};

      performConversionCycles(random, rotationsToGoThrough, 1000, 1000, EPSILON);
   }

   private void performConversionCycles(Random random, AllRotations[] rotationsToGoThrough, int numberOfCycles, int numberOfConversionsPerCycle, double epsilon)
   {
      boolean goingThroughYawPitchRoll = false;
      for (AllRotations allRotations : rotationsToGoThrough)
      {
         if (allRotations == AllRotations.YAW_PITCH_ROLL)
            goingThroughYawPitchRoll = true;
      }

      for (int cycle = 0; cycle < numberOfCycles; cycle++)
      {
         int initialIndex = random.nextInt(rotationsToGoThrough.length);
         AllRotations initialRotationType = rotationsToGoThrough[initialIndex];

         Matrix3DReadOnly originalMatrix = null;
         AxisAngleReadOnly originalAxisAngle = null;
         QuaternionReadOnly<?> originalQuaternion = null;
         Vector3DReadOnly originalRotationVector = null;
         double[] originalYawPitchRoll = null;

         switch (initialRotationType)
         {
         case MATRIX:
            originalMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            initialRotationType.rotationHolder = new RotationMatrix(originalMatrix);
            break;
         case AXISANGLE:
            originalAxisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            initialRotationType.rotationHolder = new AxisAngle(originalAxisAngle);
            break;
         case QUATERNION:
            originalQuaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
            initialRotationType.rotationHolder = new Quaternion(originalQuaternion);
            break;
         case VECTOR:
            originalRotationVector = GeometryBasicsRandomTools.generateRandomRotationVector(random);
            initialRotationType.rotationHolder = new Vector3D(originalRotationVector);
            break;
         case YAW_PITCH_ROLL:
            originalYawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);
            initialRotationType.rotationHolder = new double[3];
            System.arraycopy(originalYawPitchRoll, 0, initialRotationType.rotationHolder, 0, 3);
            break;
         default:
            throw AllRotations.exception(initialRotationType);
         }

         if (originalMatrix == null)
            originalMatrix = initialRotationType.convertToMatrix();
         if (originalAxisAngle == null)
            originalAxisAngle = initialRotationType.convertToAxisAngle();
         if (originalQuaternion == null)
            originalQuaternion = initialRotationType.convertToQuaternion();
         if (originalRotationVector == null)
            originalRotationVector = initialRotationType.convertToRotationVector();
         if (originalYawPitchRoll == null)
            originalYawPitchRoll = initialRotationType.convertToYawPitchRoll();

         boolean isYawPitchRollNaN = GeometryBasicsTools.containsNaN(originalYawPitchRoll);

         AllRotations previousRotationType = initialRotationType;

         if (DEBUG)
         {
            System.out.println("-------------------------------------------------------");
            System.out.println("New Cycle! (cycle #" + cycle + ")");
            System.out.println("-------------------------------------------------------");
            System.out.println("Originals (starting cycle with " + initialRotationType + "):");
            System.out.println(AllRotations.MATRIX);
            System.out.println(originalMatrix);
            System.out.println(AllRotations.AXISANGLE);
            System.out.println(originalAxisAngle);
            System.out.println(AllRotations.QUATERNION);
            System.out.println(originalQuaternion);
            System.out.println(AllRotations.VECTOR);
            System.out.println(originalRotationVector);
            System.out.println(AllRotations.YAW_PITCH_ROLL);
            System.out.println(Arrays.toString(originalYawPitchRoll));
         }

         for (int i = 0; i < numberOfConversionsPerCycle; i++)
         {
            int nextIndex;
            if (isYawPitchRollNaN && goingThroughYawPitchRoll)
               nextIndex = random.nextInt(rotationsToGoThrough.length - 2);
            else
               nextIndex = random.nextInt(rotationsToGoThrough.length - 1);

            AllRotations nextRotationType = rotationsToGoThrough[nextIndex];
            if (nextIndex >= previousRotationType.ordinal())
               nextRotationType = rotationsToGoThrough[nextIndex + 1];

            nextRotationType.rotationHolder = previousRotationType.convertTo(nextRotationType);

            if (DEBUG)
            {
               System.out.println(nextRotationType + " current iteration: " + i);
               if (nextRotationType == AllRotations.YAW_PITCH_ROLL)
                  System.out.println(Arrays.toString((double[]) nextRotationType.rotationHolder));
               else
                  System.out.println(nextRotationType.rotationHolder);
            }

            switch (nextRotationType)
            {
            case MATRIX:
               GeometryBasicsTestTools.assertMatrix3DEquals(originalMatrix, (Matrix3DReadOnly) nextRotationType.rotationHolder, epsilon);
               break;
            case AXISANGLE:
               GeometryBasicsTestTools.assertAxisAngleEqualsSmart(originalAxisAngle, (AxisAngleReadOnly) nextRotationType.rotationHolder, epsilon);
               break;
            case QUATERNION:
               GeometryBasicsTestTools.assertQuaternionEqualsSmart(originalQuaternion, (QuaternionReadOnly<?>) nextRotationType.rotationHolder, epsilon);
               break;
            case VECTOR:
               GeometryBasicsTestTools.assertRotationVectorEquals(originalRotationVector, (Vector3DReadOnly) nextRotationType.rotationHolder, epsilon);
               break;
            case YAW_PITCH_ROLL:
               GeometryBasicsTestTools.assertYawPitchRollEquals(originalYawPitchRoll, (double[]) nextRotationType.rotationHolder, epsilon);
            default:
               break;
            }

            previousRotationType = nextRotationType;
         }
      }
   }

   private enum AllRotations
   {
      MATRIX(new RotationMatrix()), AXISANGLE(new AxisAngle()), QUATERNION(new Quaternion()), VECTOR(new Vector3D()), YAW_PITCH_ROLL(new double[3]);

      Object rotationHolder;

      AllRotations(Object rotationHolder)
      {
         this.rotationHolder = rotationHolder;
      }

      Object convertTo(AllRotations rotationTypeToConvertTo)
      {
         switch (rotationTypeToConvertTo)
         {
         case MATRIX:
            return convertToMatrix();
         case AXISANGLE:
            return convertToAxisAngle();
         case QUATERNION:
            return convertToQuaternion();
         case VECTOR:
            return convertToRotationVector();
         case YAW_PITCH_ROLL:
            return convertToYawPitchRoll();
         default:
            throw exception(this);
         }
      }

      Matrix3DReadOnly convertToMatrix()
      {
         RotationMatrix matrix = new RotationMatrix();
         switch (this)
         {
         case MATRIX:
            matrix.set((Matrix3DReadOnly) rotationHolder);
            break;
         case AXISANGLE:
            RotationMatrixConversion.convertAxisAngleToMatrix((AxisAngleReadOnly) rotationHolder, matrix);
            break;
         case QUATERNION:
            RotationMatrixConversion.convertQuaternionToMatrix((QuaternionReadOnly<?>) rotationHolder, matrix);
            break;
         case VECTOR:
            RotationMatrixConversion.convertRotationVectorToMatrix((Vector3DReadOnly) rotationHolder, matrix);
            break;
         case YAW_PITCH_ROLL:
            RotationMatrixConversion.convertYawPitchRollToMatrix((double[]) rotationHolder, matrix);
            break;
         default:
            throw exception(this);
         }
         return matrix;
      }

      AxisAngleReadOnly convertToAxisAngle()
      {
         AxisAngle axisAngle = new AxisAngle();
         switch (this)
         {
         case MATRIX:
            AxisAngleConversion.convertMatrixToAxisAngle((RotationMatrixReadOnly<?>) rotationHolder, axisAngle);
            break;
         case AXISANGLE:
            axisAngle.set((AxisAngleReadOnly) rotationHolder);
            break;
         case QUATERNION:
            AxisAngleConversion.convertQuaternionToAxisAngle((QuaternionReadOnly<?>) rotationHolder, axisAngle);
            break;
         case VECTOR:
            AxisAngleConversion.convertRotationVectorToAxisAngle((Vector3DReadOnly) rotationHolder, axisAngle);
            break;
         case YAW_PITCH_ROLL:
            AxisAngleConversion.convertYawPitchRollToAxisAngle((double[]) rotationHolder, axisAngle);
            break;
         default:
            throw exception(this);
         }
         return axisAngle;
      }

      QuaternionReadOnly<?> convertToQuaternion()
      {
         Quaternion quaternion = new Quaternion();
         switch (this)
         {
         case MATRIX:
            QuaternionConversion.convertMatrixToQuaternion((RotationMatrixReadOnly<?>) rotationHolder, quaternion);
            break;
         case AXISANGLE:
            QuaternionConversion.convertAxisAngleToQuaternion((AxisAngleReadOnly) rotationHolder, quaternion);
            break;
         case QUATERNION:
            quaternion.set((QuaternionReadOnly<?>) rotationHolder);
            break;
         case VECTOR:
            QuaternionConversion.convertRotationVectorToQuaternion((Vector3DReadOnly) rotationHolder, quaternion);
            break;
         case YAW_PITCH_ROLL:
            QuaternionConversion.convertYawPitchRollToQuaternion((double[]) rotationHolder, quaternion);
            break;
         default:
            throw exception(this);
         }
         return quaternion;
      }

      Vector3DReadOnly convertToRotationVector()
      {
         Vector3D rotationVector = new Vector3D();
         switch (this)
         {
         case MATRIX:
            RotationVectorConversion.convertMatrixToRotationVector((RotationMatrixReadOnly<?>) rotationHolder, rotationVector);
            break;
         case AXISANGLE:
            RotationVectorConversion.convertAxisAngleToRotationVector((AxisAngleReadOnly) rotationHolder, rotationVector);
            break;
         case QUATERNION:
            RotationVectorConversion.convertQuaternionToRotationVector((QuaternionReadOnly<?>) rotationHolder, rotationVector);
            break;
         case VECTOR:
            rotationVector.set((Vector3DReadOnly) rotationHolder);
            break;
         case YAW_PITCH_ROLL:
            RotationVectorConversion.convertYawPitchRollToRotationVector((double[]) rotationHolder, rotationVector);
            break;
         default:
            throw exception(this);
         }
         return rotationVector;
      }

      double[] convertToYawPitchRoll()
      {
         double[] yawPitchRoll = new double[3];
         switch (this)
         {
         case MATRIX:
            YawPitchRollConversion.convertMatrixToYawPitchRoll((RotationMatrixReadOnly<?>) rotationHolder, yawPitchRoll);
            break;
         case AXISANGLE:
            YawPitchRollConversion.convertAxisAngleToYawPitchRoll((AxisAngleReadOnly) rotationHolder, yawPitchRoll);
            break;
         case QUATERNION:
            YawPitchRollConversion.convertQuaternionToYawPitchRoll((QuaternionReadOnly<?>) rotationHolder, yawPitchRoll);
            break;
         case VECTOR:
            YawPitchRollConversion.convertRotationVectorToYawPitchRoll((Vector3DReadOnly) rotationHolder, yawPitchRoll);
            break;
         case YAW_PITCH_ROLL:
            System.arraycopy(rotationHolder, 0, yawPitchRoll, 0, 3);
            break;
         default:
            throw exception(this);
         }
         return yawPitchRoll;
      }

      private static RuntimeException exception(AllRotations badEnumValue)
      {
         return new RuntimeException("Should not get there, enum value: " + badEnumValue);
      }
   }
}
