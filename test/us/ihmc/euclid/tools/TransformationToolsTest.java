package us.ihmc.euclid.tools;

import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class TransformationToolsTest
{
   private static final int NUMBER_OF_ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testTransformationsWithMatrix3DReadOnly() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Matrix3D matrix3D = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Tuple3DBasics tupleExpected = new Point3D();
         matrix3D.transform(tupleOriginal, tupleExpected);

         Tuple3DBasics tupleActual = new Point3D();
         tupleActual.setX(TransformationTools.computeTransformedX(matrix3D, false, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(matrix3D, false, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(matrix3D, false, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

         Matrix3D transposedMatrix3D = new Matrix3D(matrix3D);
         transposedMatrix3D.transpose();
         transposedMatrix3D.transform(tupleOriginal, tupleExpected);
         tupleActual.setX(TransformationTools.computeTransformedX(matrix3D, true, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(matrix3D, true, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(matrix3D, true, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithQuaternionReadOnly() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionReadOnly quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Tuple3DBasics tupleExpected = new Point3D();
         quaternion.transform(tupleOriginal, tupleExpected);

         Tuple3DBasics tupleActual = new Point3D();
         tupleActual.setX(TransformationTools.computeTransformedX(quaternion, false, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(quaternion, false, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(quaternion, false, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

         Quaternion conjugatedQuaternion = new Quaternion(quaternion);
         conjugatedQuaternion.conjugate();
         conjugatedQuaternion.transform(tupleOriginal, tupleExpected);
         tupleActual.setX(TransformationTools.computeTransformedX(quaternion, true, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(quaternion, true, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(quaternion, true, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test for Point3DReadOnly
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Point3DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics pointExpected = new Point3D();
         transform.transform(pointOriginal, pointExpected);

         Point3DBasics pointActual = new Point3D();
         pointActual.setX(TransformationTools.computeTransformedX(transform, false, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, false, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, false, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);

         RigidBodyTransform invertedTransform = new RigidBodyTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(pointOriginal, pointExpected);
         pointActual.setX(TransformationTools.computeTransformedX(transform, true, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, true, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, true, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test for Vector3DReadOnly
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DBasics vectorExpected = new Vector3D();
         transform.transform(vectorOriginal, vectorExpected);

         Vector3DBasics vectorActual = new Vector3D();
         vectorActual.setX(TransformationTools.computeTransformedX(transform, false, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, false, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, false, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);

         RigidBodyTransform invertedTransform = new RigidBodyTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(vectorOriginal, vectorExpected);
         vectorActual.setX(TransformationTools.computeTransformedX(transform, true, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, true, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, true, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test for Point3DReadOnly
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         Point3DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics pointExpected = new Point3D();
         transform.transform(pointOriginal, pointExpected);

         Point3DBasics pointActual = new Point3D();
         pointActual.setX(TransformationTools.computeTransformedX(transform, false, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, false, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, false, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);

         QuaternionBasedTransform invertedTransform = new QuaternionBasedTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(pointOriginal, pointExpected);
         pointActual.setX(TransformationTools.computeTransformedX(transform, true, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, true, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, true, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test for Vector3DReadOnly
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         Vector3DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DBasics vectorExpected = new Vector3D();
         transform.transform(vectorOriginal, vectorExpected);

         Vector3DBasics vectorActual = new Vector3D();
         vectorActual.setX(TransformationTools.computeTransformedX(transform, false, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, false, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, false, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);

         QuaternionBasedTransform invertedTransform = new QuaternionBasedTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(vectorOriginal, vectorExpected);
         vectorActual.setX(TransformationTools.computeTransformedX(transform, true, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, true, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, true, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test for Point3DReadOnly
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Point3DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics pointExpected = new Point3D();
         transform.transform(pointOriginal, pointExpected);

         Point3DBasics pointActual = new Point3D();
         pointActual.setX(TransformationTools.computeTransformedX(transform, false, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, false, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, false, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);

         transform.inverseTransform(pointOriginal, pointExpected);
         pointActual.setX(TransformationTools.computeTransformedX(transform, true, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, true, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, true, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test for Vector3DReadOnly
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DBasics vectorExpected = new Vector3D();
         transform.transform(vectorOriginal, vectorExpected);

         Vector3DBasics vectorActual = new Vector3D();
         vectorActual.setX(TransformationTools.computeTransformedX(transform, false, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, false, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, false, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);

         transform.inverseTransform(vectorOriginal, vectorExpected);
         vectorActual.setX(TransformationTools.computeTransformedX(transform, true, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, true, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, true, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }

   @Test
   public void testApplyRotationMatrices() throws Exception
   {
      Random random = new Random(234235);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test with random matrices, should throw an exception when checkIfTransformationInXYplane = true
         RotationMatrix m1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix m2 = EuclidCoreRandomTools.nextRotationMatrix(random);
         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics tupleTransformed = EuclidCoreRandomTools.nextPoint2D(random);
         boolean checkIfTransformationInXYplane = true;
         try
         {
            TransformationTools.applyRotationMatrices(m1, m2, tupleOriginal, tupleTransformed, checkIfTransformationInXYplane);
            fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
         }
         catch (NotAMatrix2DException e)
         {
            // good
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test with random matrices, test the transformation when checkIfTransformationInXYplane = false
         RotationMatrix m1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix m2 = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix m1TimesM2 = new RotationMatrix(m1);
         m1TimesM2.multiply(m2);
         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics expected = new Point2D();
         boolean checkIfTransformationInXYplane = false;
         m1TimesM2.transform(tupleOriginal, expected, checkIfTransformationInXYplane);
         TransformationTools.applyRotationMatrices(m1, m2, tupleOriginal, actual, checkIfTransformationInXYplane);

         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test with random matrices which are not 2D matrices but when multiplied represent a 2D matrix
         RotationMatrix m1 = new RotationMatrix();
         RotationMatrix m2 = new RotationMatrix();
         m1.setToYawMatrix(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         m2.setToYawMatrix(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         RotationMatrix m1TimesM2 = new RotationMatrix(m1);
         m1TimesM2.multiply(m2);

         RotationMatrix m3D = EuclidCoreRandomTools.nextRotationMatrix(random);
         m1.multiply(m3D);
         m2.preMultiplyTransposeOther(m3D);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics expected = new Point2D();
         boolean checkIfTransformationInXYplane = true;
         m1TimesM2.transform(tupleOriginal, expected, checkIfTransformationInXYplane);
         TransformationTools.applyRotationMatrices(m1, m2, tupleOriginal, actual, checkIfTransformationInXYplane);

         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testApplyRigidBodyTransforms() throws Exception
   {
      Random random = new Random(78654);

      { // Test applyRigidBodyTransforms(RigidBodyTransform m1, RigidBodyTransform m2, Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformationInXYplane)

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         { // Test with random transforms, should throw an exception when checkIfTransformationInXYplane = true
            RigidBodyTransform m1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            RigidBodyTransform m2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            Point2DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
            Point2DBasics pointTransformed = EuclidCoreRandomTools.nextPoint2D(random);
            boolean checkIfTransformationInXYplane = true;
            try
            {
               TransformationTools.applyRigidBodyTransforms(m1, m2, pointOriginal, pointTransformed, checkIfTransformationInXYplane);
               fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
            }
            catch (NotAMatrix2DException e)
            {
               // good
            }
         }

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         { // Test with random transforms, test the transformation when checkIfTransformationInXYplane = false
            RigidBodyTransform m1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            RigidBodyTransform m2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            RigidBodyTransform m1TimesM2 = new RigidBodyTransform(m1);
            m1TimesM2.multiply(m2);
            Point2DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
            Point2DBasics actual = EuclidCoreRandomTools.nextPoint2D(random);
            Point2DBasics expected = new Point2D();
            boolean checkIfTransformationInXYplane = false;
            m1TimesM2.transform(pointOriginal, expected, checkIfTransformationInXYplane);
            TransformationTools.applyRigidBodyTransforms(m1, m2, pointOriginal, actual, checkIfTransformationInXYplane);

            EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         }

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         { // Test with random transforms which are not 2D transforms but when multiplied represent a 2D transform
            RigidBodyTransform m1 = new RigidBodyTransform();
            RigidBodyTransform m2 = new RigidBodyTransform();
            m1.setRotationYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
            m1.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
            m2.setRotationYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
            m2.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
            RigidBodyTransform m1TimesM2 = new RigidBodyTransform(m1);
            m1TimesM2.multiply(m2);

            RigidBodyTransform m3D = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            m1.multiply(m3D);
            m2.preMultiplyInvertOther(m3D);

            Point2DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
            Point2DBasics actual = EuclidCoreRandomTools.nextPoint2D(random);
            Point2DBasics expected = new Point2D();
            boolean checkIfTransformationInXYplane = true;
            m1TimesM2.transform(pointOriginal, expected, checkIfTransformationInXYplane);
            TransformationTools.applyRigidBodyTransforms(m1, m2, pointOriginal, actual, checkIfTransformationInXYplane);

            EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         }
      }

      { // Test applyRigidBodyTransforms(RigidBodyTransform m1, RigidBodyTransform m2, Vector2DReadOnly pointOriginal, Vector2DBasics pointTransformed, boolean checkIfTransformationInXYplane)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         { // Test with random transforms, should throw an exception when checkIfTransformationInXYplane = true
            RigidBodyTransform m1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            RigidBodyTransform m2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            Vector2DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector2D(random);
            Vector2DBasics vectorTransformed = EuclidCoreRandomTools.nextVector2D(random);
            boolean checkIfTransformationInXYplane = true;
            try
            {
               TransformationTools.applyRigidBodyTransforms(m1, m2, vectorOriginal, vectorTransformed, checkIfTransformationInXYplane);
               fail("Should have thrown a " + NotAMatrix2DException.class.getSimpleName());
            }
            catch (NotAMatrix2DException e)
            {
               // good
            }
         }

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         { // Test with random transforms, test the transformation when checkIfTransformationInXYplane = false
            RigidBodyTransform m1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            RigidBodyTransform m2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            RigidBodyTransform m1TimesM2 = new RigidBodyTransform(m1);
            m1TimesM2.multiply(m2);
            Vector2DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector2D(random);
            Vector2DBasics actual = EuclidCoreRandomTools.nextVector2D(random);
            Vector2DBasics expected = new Vector2D();
            boolean checkIfTransformationInXYplane = false;
            m1TimesM2.transform(vectorOriginal, expected, checkIfTransformationInXYplane);
            TransformationTools.applyRigidBodyTransforms(m1, m2, vectorOriginal, actual, checkIfTransformationInXYplane);

            EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         }

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         { // Test with random transforms which are not 2D transforms but when multiplied represent a 2D transform
            RigidBodyTransform m1 = new RigidBodyTransform();
            RigidBodyTransform m2 = new RigidBodyTransform();
            m1.setRotationYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
            m1.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
            m2.setRotationYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
            m2.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
            RigidBodyTransform m1TimesM2 = new RigidBodyTransform(m1);
            m1TimesM2.multiply(m2);

            RigidBodyTransform m3D = EuclidCoreRandomTools.nextRigidBodyTransform(random);
            m1.multiply(m3D);
            m2.preMultiplyInvertOther(m3D);

            Vector2DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector2D(random);
            Vector2DBasics actual = EuclidCoreRandomTools.nextVector2D(random);
            Vector2DBasics expected = new Vector2D();
            boolean checkIfTransformationInXYplane = true;
            m1TimesM2.transform(vectorOriginal, expected, checkIfTransformationInXYplane);
            TransformationTools.applyRigidBodyTransforms(m1, m2, vectorOriginal, actual, checkIfTransformationInXYplane);

            EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
         }
      }
   }
}
