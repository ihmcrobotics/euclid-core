package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Tuple3D;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class Point3DTest extends Tuple3DTest
{
   @Test
   public void testPoint()
   {
      Random random = new Random(621541L);
      Point3D point = new Point3D();

      { // Test Point()
         Assert.assertTrue(0 == point.getX());
         Assert.assertTrue(0 == point.getY());
         Assert.assertTrue(0 == point.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         point = new Point3D(newX, newY, newZ);

         Assert.assertTrue(newX == point.getX());
         Assert.assertTrue(newY == point.getY());
         Assert.assertTrue(newZ == point.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point(double[] pointArray)
         double[] randomPointArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] copyRandomPointArray = new double[3];
         copyRandomPointArray[0] = randomPointArray[0];
         copyRandomPointArray[1] = randomPointArray[1];
         copyRandomPointArray[2] = randomPointArray[2];

         Point3D pointArray = new Point3D(randomPointArray);

         Assert.assertTrue(randomPointArray[0] == pointArray.getX());
         Assert.assertTrue(randomPointArray[1] == pointArray.getY());
         Assert.assertTrue(randomPointArray[2] == pointArray.getZ());

         Assert.assertTrue(copyRandomPointArray[0] == randomPointArray[0]);
         Assert.assertTrue(copyRandomPointArray[1] == randomPointArray[1]);
         Assert.assertTrue(copyRandomPointArray[2] == randomPointArray[2]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point(TupleBasics tuple)
         Point3D point2 = new Point3D();
         point2.setX(random.nextDouble());
         point2.setY(random.nextDouble());
         point2.setZ(random.nextDouble());

         point = new Point3D((Tuple3DReadOnly) point2);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
         Assert.assertTrue(point.getZ() == point2.getZ());
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(65465131L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         Point3D point = new Point3D(newX, newY, newZ);

         Point3D point2 = new Point3D();
         point2.set(point);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
         Assert.assertTrue(point.getZ() == point2.getZ());
      }
   }

   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double newX1 = random.nextDouble();
         double newY1 = random.nextDouble();
         double newZ1 = random.nextDouble();

         double newX2 = random.nextDouble();
         double newY2 = random.nextDouble();
         double newZ2 = random.nextDouble();

         Point3D point = new Point3D(newX1, newY1, newZ1);
         Point3D point2 = new Point3D(newX2, newY2, newZ2);

         double distance = Math.sqrt(point.distanceSquared((Point3DReadOnly) point2));
         double distance2 = point.distance(point2);

         Assert.assertTrue(distance == distance2);
      }
   }

   @Test
   @Ignore
   public void testDistanceL1()
   {
      fail("Not yet implemented");
   }

   @Test
   @Ignore
   public void testDistanceLinf()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testDistanceSquared()
   {
      Random random = new Random(654135L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double newX1 = random.nextDouble();
         double newY1 = random.nextDouble();
         double newZ1 = random.nextDouble();

         double newX2 = random.nextDouble();
         double newY2 = random.nextDouble();
         double newZ2 = random.nextDouble();

         Point3D point = new Point3D(newX1, newY1, newZ1);
         Point3D point2 = new Point3D(newX2, newY2, newZ2);

         double distance = point.distanceSquared((Point3DReadOnly) point2);
         double dx = point.getX() - point2.getX();
         double dy = point.getY() - point2.getY();
         double dz = point.getZ() - point2.getZ();
         double expectedDistance = dx * dx + dy * dy + dz * dz;

         Assert.assertTrue(distance == expectedDistance);
      }
   }

   @Test
   @Ignore
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3D point1 = new Point3D();
         Point3D point2 = new Point3D();

         double epsilon = random.nextDouble();

         point1.setX(random.nextDouble());
         point1.setY(random.nextDouble());
         point1.setZ(random.nextDouble());

         point2.setX(point1.getX() + 0.1 * epsilon);
         point2.setY(point1.getY() + 0.1 * epsilon);
         point2.setZ(point1.getZ() + 0.1 * epsilon);
         assertTrue(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() - 0.1 * epsilon);
         point2.setY(point1.getY() - 0.1 * epsilon);
         point2.setZ(point1.getZ() - 0.1 * epsilon);
         assertTrue(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() - 1.1 * epsilon);
         point2.setY(point1.getY() - 0.1 * epsilon);
         point2.setZ(point1.getZ() - 0.1 * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() - 0.1 * epsilon);
         point2.setY(point1.getY() - 1.1 * epsilon);
         point2.setZ(point1.getZ() - 0.1 * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() - 0.1 * epsilon);
         point2.setY(point1.getY() - 0.1 * epsilon);
         point2.setZ(point1.getZ() - 1.1 * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() + 1.1 * epsilon);
         point2.setY(point1.getY() + 0.1 * epsilon);
         point2.setZ(point1.getZ() + 0.1 * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() + 0.1 * epsilon);
         point2.setY(point1.getY() + 1.1 * epsilon);
         point2.setZ(point1.getZ() + 0.1 * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() + 0.1 * epsilon);
         point2.setY(point1.getY() + 0.1 * epsilon);
         point2.setZ(point1.getZ() + 1.1 * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() + epsilon - 1.0e-15);
         point2.setY(point1.getY() + epsilon - 1.0e-15);
         point2.setZ(point1.getZ() + epsilon - 1.0e-15);
         assertTrue(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX() - epsilon + 1.0e-15);
         point2.setY(point1.getY() - epsilon + 1.0e-15);
         point2.setZ(point1.getZ() - epsilon + 1.0e-15);
         assertTrue(point1.epsilonEquals(point2, epsilon));
      }
   }

   @Override
   public Tuple3D createEmptyTuple()
   {
      return new Point3D();
   }
}
