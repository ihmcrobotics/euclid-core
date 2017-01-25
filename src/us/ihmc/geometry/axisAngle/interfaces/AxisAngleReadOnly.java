package us.ihmc.geometry.axisAngle.interfaces;

/**
 * Simplest interface defining the reading API for an axis-angle object.
 * An axis-angle is used to represent a 3D orientation by a unitary axis
 * of components (x, y, z) and an angle of rotation usually expressed in radians.
 * 
 * @author Sylvain
 */
public interface AxisAngleReadOnly
{
   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    * @return the angle.
    */
   double getAngle();

   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    * @return the x-component of the unitary axis.
    */
   double getX();

   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    * @return the y-component of the unitary axis.
    */
   double getY();

   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    * @return the z-component of the unitary axis.
    */
   double getZ();

}