package us.ihmc.geometry.testingTools;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Tuple3D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class GeometryBasicsRandomToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testGenerateRandomVector()
   {
      for (int i = 0; i < 10000; i++)
      {
         Random random = new Random(16451L);

         Vector3D vector = new Vector3D();
         Vector3D zeroVector = new Vector3D();
         Vector3D min = new Vector3D(-1, -1, -1);
         Vector3D max = new Vector3D(1, 1, 1);

         GeometryBasicsRandomTools.generateRandomVector(random);

         Assert.assertTrue(min.getX() <= vector.getX());
         Assert.assertTrue(vector.getX() <= max.getX());
         Assert.assertTrue(min.getY() <= vector.getY());
         Assert.assertTrue(vector.getY() <= max.getY());
         Assert.assertTrue(min.getZ() <= vector.getZ());
         Assert.assertTrue(vector.getZ() <= max.getZ());

         Assert.assertTrue(zeroVector.getX() == 0);
         Assert.assertTrue(zeroVector.getY() == 0);
         Assert.assertTrue(zeroVector.getZ() == 0);

         Assert.assertNotSame(vector, null);
      }
   }

   @Test
   public void testRandomizeTuple()
   {
      Random random = new Random(6841032L);
      Tuple3D tupleToRandomize = new Point3D();

      Tuple3D tupleToRandomizeCopy = new Point3D();
      tupleToRandomizeCopy.setX(tupleToRandomize.getX());
      tupleToRandomizeCopy.setY(tupleToRandomize.getY());
      tupleToRandomizeCopy.setZ(tupleToRandomize.getZ());

      { // Test randomize(Random random, TupleBasics tupleToRandomize)
         Tuple3D previousValue = new Point3D();
         previousValue.setToNaN();

         for (int i = 0; i < 10000; i++)
         {
            GeometryBasicsRandomTools.randomizeTuple(random, (Tuple3DBasics) tupleToRandomize);

            Assert.assertTrue(-1.0 <= tupleToRandomize.getX());
            Assert.assertTrue(tupleToRandomize.getX() <= 1.0);
            Assert.assertTrue(-1.0 <= tupleToRandomize.getY());
            Assert.assertTrue(tupleToRandomize.getY() <= 1.0);
            Assert.assertTrue(-1.0 <= tupleToRandomize.getZ());
            Assert.assertTrue(tupleToRandomize.getZ() <= 1.0);

            Assert.assertFalse(tupleToRandomize.epsilonEquals(previousValue, 1.0e-10));
            previousValue.set(tupleToRandomize);
         }
      }

      for (int i = 0; i < 10000; i++)
      { // Test randomize(Random random, TupleBasics minMax, TupleBasics tupleToRandomize)
         Tuple3D minMax = new Point3D();

         minMax.setX(random.nextDouble());
         minMax.setY(random.nextDouble());
         minMax.setZ(random.nextDouble());

         GeometryBasicsRandomTools.randomizeTuple(random, (Tuple3DReadOnly) minMax, (Tuple3DBasics) tupleToRandomize);

         Assert.assertTrue(tupleToRandomizeCopy.getX() - minMax.getX() <= tupleToRandomize.getX());
         Assert.assertTrue(tupleToRandomize.getX() <= tupleToRandomizeCopy.getX() + minMax.getX());
         Assert.assertTrue(tupleToRandomizeCopy.getY() - minMax.getY() <= tupleToRandomize.getY());
         Assert.assertTrue(tupleToRandomize.getY() <= tupleToRandomizeCopy.getY() + minMax.getY());
         Assert.assertTrue(tupleToRandomizeCopy.getZ() - minMax.getZ() <= tupleToRandomize.getZ());
         Assert.assertTrue(tupleToRandomize.getZ() <= tupleToRandomizeCopy.getZ() + minMax.getZ());
      }

      { // Test randomize(Random random, TupleBasics min, TupleBasics max, TupleBasics tupleToRandomize)
         Tuple3D min = new Point3D();
         Tuple3D max = new Point3D();

         for (int i = 0; i < 10000; i++)
         {
            min.setX(random.nextDouble());
            min.setY(random.nextDouble());
            min.setZ(random.nextDouble());

            max.setX(min.getX() + random.nextDouble());
            max.setY(min.getY() + random.nextDouble());
            max.setZ(min.getZ() + random.nextDouble());

            GeometryBasicsRandomTools.randomizeTuple(random, (Tuple3DReadOnly) min, (Tuple3DReadOnly) max, (Tuple3DBasics) tupleToRandomize);

            Assert.assertTrue(tupleToRandomizeCopy.getX() - min.getX() <= tupleToRandomize.getX());
            Assert.assertTrue(tupleToRandomize.getX() <= tupleToRandomizeCopy.getX() + max.getX());
            Assert.assertTrue(tupleToRandomizeCopy.getY() - min.getY() <= tupleToRandomize.getY());
            Assert.assertTrue(tupleToRandomize.getY() <= tupleToRandomizeCopy.getY() + max.getY());
            Assert.assertTrue(tupleToRandomizeCopy.getZ() - min.getZ() <= tupleToRandomize.getZ());
            Assert.assertTrue(tupleToRandomize.getZ() <= tupleToRandomizeCopy.getZ() + max.getZ());
         }
      }
   }

   @Test
   public void testRandomizeAxisAngle() throws Exception
   {
      Random random = new Random(54654L);
      AxisAngle axisAngle = new AxisAngle();
      AxisAngle axisAnglePrevious = new AxisAngle();
      double minMax = 1.0;
      double actualAngleMin = Double.POSITIVE_INFINITY;
      double actualAngleMax = Double.NEGATIVE_INFINITY;

      axisAnglePrevious.setToNaN();

      for (int i = 0; i < 100000; i++)
      {
         GeometryBasicsRandomTools.randomizeAxisAngle(random, minMax, axisAngle);
         assertTrue(Math.abs(axisAngle.getAngle()) < minMax);
         double uNorm = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());
         assertTrue(Math.abs(uNorm - 1.0) < EPSILON);
         boolean hasChanged = axisAngle.getX() != axisAnglePrevious.getX();
         hasChanged &= axisAngle.getY() != axisAnglePrevious.getY();
         hasChanged &= axisAngle.getZ() != axisAnglePrevious.getZ();
         hasChanged &= axisAngle.getAngle() != axisAnglePrevious.getAngle();
         assertTrue(hasChanged);

         actualAngleMin = Math.min(actualAngleMin, axisAngle.getAngle());
         actualAngleMax = Math.max(actualAngleMax, axisAngle.getAngle());

         axisAnglePrevious.setX(axisAngle.getX());
         axisAnglePrevious.setY(axisAngle.getY());
         axisAnglePrevious.setZ(axisAngle.getZ());
         axisAnglePrevious.setAngle(axisAngle.getAngle());
      }

      assertTrue(actualAngleMax > 0.75 * minMax);
      assertTrue(actualAngleMin < -0.75 * minMax);

      axisAnglePrevious.setToNaN();

      for (int i = 0; i < 100000; i++)
      {
         GeometryBasicsRandomTools.randomizeAxisAngle(random, axisAngle);
         assertTrue(Math.abs(axisAngle.getAngle()) < Math.PI);
         double uNorm = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());
         assertTrue(Math.abs(uNorm - 1.0) < EPSILON);
         boolean hasChanged = axisAngle.getX() != axisAnglePrevious.getX();
         hasChanged &= axisAngle.getY() != axisAnglePrevious.getY();
         hasChanged &= axisAngle.getZ() != axisAnglePrevious.getZ();
         hasChanged &= axisAngle.getAngle() != axisAnglePrevious.getAngle();
         assertTrue(hasChanged);

         axisAnglePrevious.setX(axisAngle.getX());
         axisAnglePrevious.setY(axisAngle.getY());
         axisAnglePrevious.setZ(axisAngle.getZ());
         axisAnglePrevious.setAngle(axisAngle.getAngle());
      }
   }
}
