package us.ihmc.euclid.tools;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * {@code TransformationTools} provides a list a methods for transforming geometry objects useful in
 * particular contexts where the result cannot be stored in an object.
 * <p>
 * Note that in common situations, the use of {@code TransformationTools} should be avoided
 * preferring the use of the 'transform' or 'applyTransform' methods provided with the concerned
 * objects. Also note that these methods are possibly more computationally expensive than their
 * respective couterparts.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class TransformationTools
{
   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code matrix}.
    * 
    * @param matrix the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix
    *           or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(Matrix3DReadOnly matrix, boolean transpose, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedX(matrix, transpose, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code matrix}.
    * 
    * @param matrix the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix
    *           or not.
    * @param x the x-coordinate of the tuple to be transformed.
    * @param y the y-coordinate of the tuple to be transformed.
    * @param z the z-coordinate of the tuple to be transformed.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(Matrix3DReadOnly matrix, boolean transpose, double x, double y, double z)
   {
      if (transpose)
         return matrix.getM00() * x + matrix.getM10() * y + matrix.getM20() * z;
      else
         return matrix.getM00() * x + matrix.getM01() * y + matrix.getM02() * z;
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code matrix}.
    * 
    * @param matrix the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix
    *           or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(Matrix3DReadOnly matrix, boolean transpose, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedY(matrix, transpose, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code matrix}.
    * 
    * @param matrix the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix
    *           or not.
    * @param x the x-coordinate of the tuple to be transformed.
    * @param y the y-coordinate of the tuple to be transformed.
    * @param z the z-coordinate of the tuple to be transformed.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(Matrix3DReadOnly matrix, boolean transpose, double x, double y, double z)
   {
      if (transpose)
         return matrix.getM01() * x + matrix.getM11() * y + matrix.getM21() * z;
      else
         return matrix.getM10() * x + matrix.getM11() * y + matrix.getM12() * z;
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code matrix}.
    * 
    * @param matrix the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix
    *           or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(Matrix3DReadOnly matrix, boolean transpose, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedZ(matrix, transpose, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code matrix}.
    * 
    * @param matrix the matrix used to transform the given tuple. Not modified.
    * @param transpose whether the operation should performed with the transpose of the given matrix
    *           or not.
    * @param x the x-coordinate of the tuple to be transformed.
    * @param y the y-coordinate of the tuple to be transformed.
    * @param z the z-coordinate of the tuple to be transformed.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(Matrix3DReadOnly matrix, boolean transpose, double x, double y, double z)
   {
      if (transpose)
         return matrix.getM02() * x + matrix.getM12() * y + matrix.getM22() * z;
      else
         return matrix.getM20() * x + matrix.getM21() * y + matrix.getM22() * z;
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code quaternion}.
    * 
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate whether the operation should performed with the conjugate of the given
    *           quaternion or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionReadOnly quaternion, boolean conjugate, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedX(quaternion, conjugate, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code quaternion}.
    * 
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate whether the operation should performed with the conjugate of the given
    *           quaternion or not.
    * @param x the x-coordinate of the tuple to be transformed.
    * @param y the y-coordinate of the tuple to be transformed.
    * @param z the z-coordinate of the tuple to be transformed.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionReadOnly quaternion, boolean conjugate, double x, double y, double z)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugate)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < QuaternionTools.EPS)
      {
         return x;
      }

      norm = 1.0 / norm;
      double crossX = qy * z - qz * y;
      double crossY = qz * x - qx * z;
      double crossZ = qx * y - qy * x;

      return x + 2.0 * (qs * crossX + qy * crossZ - qz * crossY) * norm * norm;
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code quaternion}.
    * 
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate whether the operation should performed with the conjugate of the given
    *           quaternion or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionReadOnly quaternion, boolean conjugate, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedY(quaternion, conjugate, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code quaternion}.
    * 
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate whether the operation should performed with the conjugate of the given
    *           quaternion or not.
    * @param x the x-coordinate of the tuple to be transformed.
    * @param y the y-coordinate of the tuple to be transformed.
    * @param z the z-coordinate of the tuple to be transformed.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionReadOnly quaternion, boolean conjugate, double x, double y, double z)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugate)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < QuaternionTools.EPS)
      {
         return y;
      }

      norm = 1.0 / norm;
      double crossX = qy * z - qz * y;
      double crossY = qz * x - qx * z;
      double crossZ = qx * y - qy * x;

      return y + 2.0 * (qs * crossY + qz * crossX - qx * crossZ) * norm * norm;
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code quaternion}.
    * 
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate whether the operation should performed with the conjugate of the given
    *           quaternion or not.
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionReadOnly quaternion, boolean conjugate, Tuple3DReadOnly tupleOriginal)
   {
      return computeTransformedZ(quaternion, conjugate, tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ());
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code tupleOriginal} by {@code quaternion}.
    * 
    * @param quaternion the quaternion used to transform the given tuple. Not modified.
    * @param conjugate whether the operation should performed with the conjugate of the given
    *           quaternion or not.
    * @param x the x-coordinate of the tuple to be transformed.
    * @param y the y-coordinate of the tuple to be transformed.
    * @param z the z-coordinate of the tuple to be transformed.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionReadOnly quaternion, boolean conjugate, double x, double y, double z)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugate)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = quaternion.norm();

      if (norm < QuaternionTools.EPS)
      {
         return z;
      }

      norm = 1.0 / norm;
      double crossX = qy * z - qz * y;
      double crossY = qz * x - qx * z;
      double crossZ = qx * y - qy * x;

      return z + 2.0 * (qs * crossZ + qx * crossY - qy * crossX) * norm * norm;
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code rigidBodyTransform}.
    * 
    * @param rigidBodyTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(RigidBodyTransform rigidBodyTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - rigidBodyTransform.getTranslationX();
         double y = pointOriginal.getY() - rigidBodyTransform.getTranslationY();
         double z = pointOriginal.getZ() - rigidBodyTransform.getTranslationZ();
         return computeTransformedX(rigidBodyTransform.getRotationMatrix(), invert, x, y, z);
      }
      else
      {
         return rigidBodyTransform.getTranslationX() + computeTransformedX(rigidBodyTransform.getRotationMatrix(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code rigidBodyTransform}.
    * 
    * @param rigidBodyTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(RigidBodyTransform rigidBodyTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - rigidBodyTransform.getTranslationX();
         double y = pointOriginal.getY() - rigidBodyTransform.getTranslationY();
         double z = pointOriginal.getZ() - rigidBodyTransform.getTranslationZ();
         return computeTransformedY(rigidBodyTransform.getRotationMatrix(), invert, x, y, z);
      }
      else
      {
         return rigidBodyTransform.getTranslationY() + computeTransformedY(rigidBodyTransform.getRotationMatrix(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code rigidBodyTransform}.
    * 
    * @param rigidBodyTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(RigidBodyTransform rigidBodyTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - rigidBodyTransform.getTranslationX();
         double y = pointOriginal.getY() - rigidBodyTransform.getTranslationY();
         double z = pointOriginal.getZ() - rigidBodyTransform.getTranslationZ();
         return computeTransformedZ(rigidBodyTransform.getRotationMatrix(), invert, x, y, z);
      }
      else
      {
         return rigidBodyTransform.getTranslationZ() + computeTransformedZ(rigidBodyTransform.getRotationMatrix(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code rigidBodyTransform}.
    * 
    * @param rigidBodyTransform the transform used to transform the given vector. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(RigidBodyTransform rigidBodyTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedX(rigidBodyTransform.getRotationMatrix(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code rigidBodyTransform}.
    * 
    * @param rigidBodyTransform the transform used to transform the given vector. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(RigidBodyTransform rigidBodyTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedY(rigidBodyTransform.getRotationMatrix(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code rigidBodyTransform}.
    * 
    * @param rigidBodyTransform the transform used to transform the given vector. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(RigidBodyTransform rigidBodyTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedZ(rigidBodyTransform.getRotationMatrix(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code quaternionBasedTransform}.
    * 
    * @param quaternionBasedTransforms the transform used to transform the given point. Not
    *           modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - quaternionBasedTransform.getTranslationX();
         double y = pointOriginal.getY() - quaternionBasedTransform.getTranslationY();
         double z = pointOriginal.getZ() - quaternionBasedTransform.getTranslationZ();
         return computeTransformedX(quaternionBasedTransform.getQuaternion(), invert, x, y, z);
      }
      else
      {
         return quaternionBasedTransform.getTranslationX() + computeTransformedX(quaternionBasedTransform.getQuaternion(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code quaternionBasedTransform}.
    * 
    * @param quaternionBasedTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - quaternionBasedTransform.getTranslationX();
         double y = pointOriginal.getY() - quaternionBasedTransform.getTranslationY();
         double z = pointOriginal.getZ() - quaternionBasedTransform.getTranslationZ();
         return computeTransformedY(quaternionBasedTransform.getQuaternion(), invert, x, y, z);
      }
      else
      {
         return quaternionBasedTransform.getTranslationY() + computeTransformedY(quaternionBasedTransform.getQuaternion(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code quaternionBasedTransform}.
    * 
    * @param quaternionBasedTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - quaternionBasedTransform.getTranslationX();
         double y = pointOriginal.getY() - quaternionBasedTransform.getTranslationY();
         double z = pointOriginal.getZ() - quaternionBasedTransform.getTranslationZ();
         return computeTransformedZ(quaternionBasedTransform.getQuaternion(), invert, x, y, z);
      }
      else
      {
         return quaternionBasedTransform.getTranslationZ() + computeTransformedZ(quaternionBasedTransform.getQuaternion(), invert, pointOriginal);
      }
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code quaternionBasedTransform}.
    * 
    * @param quaternionBasedTransform the transform used to transform the given vector. Not
    *           modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedX(quaternionBasedTransform.getQuaternion(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code quaternionBasedTransform}.
    * 
    * @param quaternionBasedTransform the transform used to transform the given vector. Not
    *           modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedY(quaternionBasedTransform.getQuaternion(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code quaternionBasedTransform}.
    * 
    * @param quaternionBasedTransform the transform used to transform the given vector. Not
    *           modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(QuaternionBasedTransform quaternionBasedTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      return computeTransformedZ(quaternionBasedTransform.getQuaternion(), invert, vectorOriginal);
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code affineTransform}.
    * 
    * @param affineTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(AffineTransform affineTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - affineTransform.getTranslationX();
         double y = pointOriginal.getY() - affineTransform.getTranslationY();
         double z = pointOriginal.getZ() - affineTransform.getTranslationZ();
         return computeTransformedX(affineTransform.getRotationMatrix(), invert, x, y, z) / affineTransform.getScaleX();
      }
      else
      {
         double x = pointOriginal.getX() * affineTransform.getScaleX();
         double y = pointOriginal.getY() * affineTransform.getScaleY();
         double z = pointOriginal.getZ() * affineTransform.getScaleZ();
         return affineTransform.getTranslationX() + computeTransformedX(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code affineTransform}.
    * 
    * @param affineTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(AffineTransform affineTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - affineTransform.getTranslationX();
         double y = pointOriginal.getY() - affineTransform.getTranslationY();
         double z = pointOriginal.getZ() - affineTransform.getTranslationZ();
         return computeTransformedY(affineTransform.getRotationMatrix(), invert, x, y, z) / affineTransform.getScaleY();
      }
      else
      {
         double x = pointOriginal.getX() * affineTransform.getScaleX();
         double y = pointOriginal.getY() * affineTransform.getScaleY();
         double z = pointOriginal.getZ() * affineTransform.getScaleZ();
         return affineTransform.getTranslationY() + computeTransformedY(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code pointOriginal} by {@code affineTransform}.
    * 
    * @param affineTransform the transform used to transform the given point. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param pointOriginal the point to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(AffineTransform affineTransform, boolean invert, Point3DReadOnly pointOriginal)
   {
      if (invert)
      {
         double x = pointOriginal.getX() - affineTransform.getTranslationX();
         double y = pointOriginal.getY() - affineTransform.getTranslationY();
         double z = pointOriginal.getZ() - affineTransform.getTranslationZ();
         return computeTransformedZ(affineTransform.getRotationMatrix(), invert, x, y, z) / affineTransform.getScaleZ();
      }
      else
      {
         double x = pointOriginal.getX() * affineTransform.getScaleX();
         double y = pointOriginal.getY() * affineTransform.getScaleY();
         double z = pointOriginal.getZ() * affineTransform.getScaleZ();
         return affineTransform.getTranslationZ() + computeTransformedZ(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the x-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code affineTransform}.
    * 
    * @param affineTransform the transform used to transform the given vector. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the x-coordinate resulting from the transformation.
    */
   public static double computeTransformedX(AffineTransform affineTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      if (invert)
      {
         return computeTransformedX(affineTransform.getRotationMatrix(), invert, vectorOriginal) / affineTransform.getScaleX();
      }
      else
      {
         double x = vectorOriginal.getX() * affineTransform.getScaleX();
         double y = vectorOriginal.getY() * affineTransform.getScaleY();
         double z = vectorOriginal.getZ() * affineTransform.getScaleZ();
         return computeTransformedX(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the y-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code affineTransform}.
    * 
    * @param affineTransform the transform used to transform the given vector. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the y-coordinate resulting from the transformation.
    */
   public static double computeTransformedY(AffineTransform affineTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      if (invert)
      {
         return computeTransformedY(affineTransform.getRotationMatrix(), invert, vectorOriginal) / affineTransform.getScaleY();
      }
      else
      {
         double x = vectorOriginal.getX() * affineTransform.getScaleX();
         double y = vectorOriginal.getY() * affineTransform.getScaleY();
         double z = vectorOriginal.getZ() * affineTransform.getScaleZ();
         return computeTransformedY(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Computes and returns the z-coordinate resulting from the transformation of
    * {@code vectorOriginal} by {@code affineTransform}.
    * 
    * @param affineTransform the transform used to transform the given vector. Not modified.
    * @param invert whether the operation should performed with the inverse of the given transform
    *           or not.
    * @param vectorOriginal the vector to be transformed. Not modified.
    * @return the z-coordinate resulting from the transformation.
    */
   public static double computeTransformedZ(AffineTransform affineTransform, boolean invert, Vector3DReadOnly vectorOriginal)
   {
      if (invert)
      {
         return computeTransformedZ(affineTransform.getRotationMatrix(), invert, vectorOriginal) / affineTransform.getScaleZ();
      }
      else
      {
         double x = vectorOriginal.getX() * affineTransform.getScaleX();
         double y = vectorOriginal.getY() * affineTransform.getScaleY();
         double z = vectorOriginal.getZ() * affineTransform.getScaleZ();
         return computeTransformedZ(affineTransform.getRotationMatrix(), invert, x, y, z);
      }
   }

   /**
    * Transforms the given tuple {@code tuple2DOriginal} by the two given matrices and stores the
    * result in {@code tuple2DTransformed}.
    * <p>
    * tuple2DTransformed = m1 * m2 * tuple2DOriginal
    * </p>
    *
    * @param m1 the first matrix use in the transformation. Not modified.
    * @param m2 the second matrix use in the transformation. Not modified.
    * @param tuple2DOriginal the tuple to transform. Not modified.
    * @param tuple2DTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the transformation
    *           {@code m1 * m2} represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the
    *            transformation {@code m1 * m2} does not represent a transformation in the XY plane.
    */
   public static void applyRotationMatrices(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Tuple2DReadOnly tuple2DOriginal, Tuple2DBasics tuple2DTransformed,
                                            boolean checkIfTransformationInXYplane)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM10() + m1.getM02() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM01() * m2.getM12() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM12() * m2.getM20();
      double m11 = m1.getM10() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM21();
      double m12 = m1.getM10() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM20() * m2.getM01() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM20() * m2.getM02() + m1.getM21() * m2.getM12() + m1.getM22() * m2.getM22();

      if (checkIfTransformationInXYplane)
      {
         if (!Matrix3DFeatures.isMatrix2D(m00, m01, m02, m10, m11, m12, m20, m21, m22, Matrix3DFeatures.EPS_CHECK_2D))
            throw new NotAMatrix2DException(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }

      double x = m00 * tuple2DOriginal.getX() + m01 * tuple2DOriginal.getY();
      double y = m10 * tuple2DOriginal.getX() + m11 * tuple2DOriginal.getY();
      tuple2DTransformed.set(x, y);
   }

   /**
    * Transforms the given {@code point2DOriginal} by the two given transforms and stores the result
    * in {@code point2DTransformed}.
    * <p>
    * point2DTransformed = transform1 * trasnform2 * point2DOriginal
    * </p>
    *
    * @param transform1 the first transform to use in the transformation. Not modified.
    * @param transform2 the second transform to use in the transformation. Not modified.
    * @param point2DOriginal the point to transform. Not modified.
    * @param point2DTransformed the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation
    *           {@code transform1.getRotationMatrix() * transfor2.getRotationMatrix()} represents a
    *           transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            {@code transform1.getRotationMatrix() * transform2.getRotationMatrix()} is not a
    *            transformation in the XY plane.
    */
   public static void applyRigidBodyTransforms(RigidBodyTransform transform1, RigidBodyTransform transform2, Point2DReadOnly point2DOriginal,
                                               Point2DBasics point2DTransformed, boolean checkIfTransformationInXYplane)
   {
      applyRotationMatrices(transform1.getRotationMatrix(), transform2.getRotationMatrix(), point2DOriginal, point2DTransformed,
                            checkIfTransformationInXYplane);
      double tx = transform1.getM00() * transform2.getM03() + transform1.getM01() * transform2.getM13() + transform1.getM02() * transform2.getM23()
            + transform1.getM03();
      double ty = transform1.getM10() * transform2.getM03() + transform1.getM11() * transform2.getM13() + transform1.getM12() * transform2.getM23()
            + transform1.getM13();
      point2DTransformed.add(tx, ty);
   }

   /**
    * Transforms the given {@code vector2DOriginal} by the two given transforms and stores the
    * result in {@code vector2DTransformed}.
    * <p>
    * vector2DTransformed = transform1 * transform2 * vector2DOriginal
    * </p>
    *
    * @param transform1 the first transform to use in the transformation. Not modified.
    * @param transform2 the second transform to use in the transformation. Not modified.
    * @param vector2DOriginal the vector to transform. Not modified.
    * @param vector2DTransformed the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation
    *           {@code transform1.getRotationMatrix() * transfor2.getRotationMatrix()} represents a
    *           transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            {@code transform1.getRotationMatrix() * transform2.getRotationMatrix()} is not a
    *            transformation in the XY plane.
    */
   public static void applyRigidBodyTransforms(RigidBodyTransform m1, RigidBodyTransform m2, Vector2DReadOnly vector2DOriginal,
                                               Vector2DBasics vector2DTransformed, boolean checkIfTransformationInXYplane)
   {
      applyRotationMatrices(m1.getRotationMatrix(), m2.getRotationMatrix(), vector2DOriginal, vector2DTransformed, checkIfTransformationInXYplane);
   }
}
