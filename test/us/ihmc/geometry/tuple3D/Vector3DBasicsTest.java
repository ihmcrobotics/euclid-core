package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;

public abstract class Vector3DBasicsTest<T extends Vector3DBasics<T>> extends Tuple3DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testLength()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double length1 = vector1.length();
         T vector2 = createEmptyTuple();
         double scalar = GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 10.0);
         vector2.setAndScale(scalar, vector1);
         double expectedLength2 = scalar * length1;
         double actualLength2 = vector2.length();
         assertEquals(expectedLength2, actualLength2, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double length1 = vector1.length();
         T vector2 = createEmptyTuple();
         double scalar = GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 10.0);
         vector2.setAndScale(scalar, vector1);
         double expectedLength2 = scalar * length1;
         double actualLength2 = vector2.lengthSquared();
         assertEquals(expectedLength2, Math.sqrt(actualLength2), 5.0 * getEpsilon());
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(5461L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         vector1.scale(GeometryBasicsRandomTools.generateRandomDouble(random, 2.0));
         Vector3DBasics<?> axis = GeometryBasicsRandomTools.generateRandomOrthogonalVector3d(random, vector1, true);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 2.0));

         double expectedDot = vector1.length() * vector2.length() * Math.cos(angle);
         double actualDot = vector1.dot(vector2);
         assertEquals(expectedDot, actualDot, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         vector1.scale(GeometryBasicsRandomTools.generateRandomDouble(random, 2.0));
         Vector3DBasics<?> axis = GeometryBasicsRandomTools.generateRandomOrthogonalVector3d(random, vector1, true);
         double expectedAngle = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, expectedAngle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 2.0));


         double actualAngle = vector1.angle(vector2);
         assertEquals(Math.abs(expectedAngle), actualAngle, 10.0 *getEpsilon());
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // cross(Vector3DReadOnly<?> other) 
         T vector1 = createRandomTuple(random);
         vector1.scale(GeometryBasicsRandomTools.generateRandomDouble(random, 2.0));
         Vector3DBasics<?> axis = GeometryBasicsRandomTools.generateRandomOrthogonalVector3d(random, vector1, true);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 2.0));

         T vector3 = createEmptyTuple();
         vector3.cross(vector1, vector2);

         double expectedCrossMagnitude = vector1.length() * vector2.length() * Math.sin(angle);
         double actualCrossMagnitude = vector3.length();
         assertEquals(expectedCrossMagnitude, actualCrossMagnitude, 10.0 * getEpsilon());

         assertEquals(0.0, vector1.dot(vector3), 10.0 * getEpsilon());
         assertEquals(0.0, vector2.dot(vector3), 10.0 * getEpsilon());
      }
   }

   // Basics part
   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize()
         T vector1 = createRandomTuple(random);
         vector1.normalize();

         double expectedLength = 1.0;
         double actualLength = vector1.length();
         assertEquals(expectedLength, actualLength, getEpsilon());

         T vector2 = createRandomTuple(random);
         vector2.normalize();
         vector1.setAndScale(GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 10.0), vector2);
         vector1.normalize();
         GeometryBasicsTestTools.assertTuple3DEquals(vector1, vector2, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize(Vector2D vector)
         T vector1 = createRandomTuple(random);
         T vector2 = createEmptyTuple();

         vector2.setAndNormalize(vector1);

         double expectedLength = 1.0;
         double actualLength = vector2.length();
         assertEquals(expectedLength, actualLength, getEpsilon());

         vector2 = createRandomTuple(random);
         vector2.normalize();
         T vector3 = createEmptyTuple();
         vector3.setAndScale(GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 10.0), vector2);
         vector1.setAndNormalize(vector3);
         GeometryBasicsTestTools.assertTuple3DEquals(vector1, vector2, getEpsilon());
      }
   }

   @Ignore
   @Test
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }
}
