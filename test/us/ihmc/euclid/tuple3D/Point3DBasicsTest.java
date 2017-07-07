package us.ihmc.euclid.tuple3D;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public abstract class Point3DBasicsTest<T extends Point3DBasics> extends Tuple3DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + translation.getZ());
         double actualDistance = p1.distance(p2);
         assertEquals(expectedDistance, actualDistance, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testDistanceSquared()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + translation.getZ());
         double actualDistanceSquared = p1.distanceSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistanceSquared, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testDistanceXY()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // With other point 3D
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + random.nextDouble());
         double actualDistance = p1.distanceXY(p2);
         assertEquals(expectedDistance, actualDistance, 5.0 * getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // With point 2D
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         Point2D p2 = new Point2D(p1.getX() + translation.getX(), p1.getY() + translation.getY());
         double actualDistance = p1.distanceXY(p2);
         assertEquals(expectedDistance, actualDistance, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testDistanceXYSquared()
   {
      Random random = new Random(65415L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // With other point 3D
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random, 1.0, 2.0);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistanceSquared = EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY(), p1.getZ() + random.nextDouble());
         double actualDistance = p1.distanceXYSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistance, 5.0 * getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // With point 2D
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random, 1.0, 2.0);
         translation.setZ(0.0);
         translation.normalize();
         double expectedDistanceSquared = EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         Point2D p2 = new Point2D(p1.getX() + translation.getX(), p1.getY() + translation.getY());
         double actualDistance = p1.distanceXYSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistance, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testDistanceFromOrigin() throws Exception
   {
      Random random = new Random(654135L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p = createTuple(translation.getX(), translation.getY(), translation.getZ());
         double actualDistance = p.distanceFromOrigin();
         assertEquals(expectedDistance, actualDistance, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testDistanceFromOriginSquared() throws Exception
   {
      Random random = new Random(654135L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p = createTuple(translation.getX(), translation.getY(), translation.getZ());
         double actualDistance = p.distanceFromOriginSquared();
         assertEquals(expectedDistanceSquared, actualDistance, 5.0 * getEpsilon());
      }
   }

   // Basics part
   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.generateRandomAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.generateRandomAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, 50.0 * getEpsilon());
      }
   }
}
