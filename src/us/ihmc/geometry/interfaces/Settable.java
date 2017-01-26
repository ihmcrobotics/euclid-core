package us.ihmc.geometry.interfaces;

/**
 * Base interface for any object that is that is settable
 * with other objects of its own type.
 * This interface also ensures basic features such as:
 * <ul>
 *    <li> {@code setToZero()} that is commonly used to reset the data in an object.
 *    <li> {@code setToNaN()} that is commonly used to invalidate an object by setting
 *     all its fields to {@link Double#NaN}.
 *    <li> {@code containsNaN()} that is commonly used to test if an object is invalid,
 *     i.e. if it contains {@link Double#NaN}.
 * </ul>
 * 
 * @author Sylvain
 *
 * @param <T> the final type of the implementation of this interface.
 */
public interface Settable<T>
{
   /**
    * Tests if this object contains at least one value equal to {@link Double#NaN}.
    * 
    * @return {@code true} if this object contains at least one value equal to {@link Double#NaN},
    *  {@code false} otherwise.
    */
   boolean containsNaN();

   /**
    * Copies the values from {@code other} into this object.
    * 
    * @param other the other object to copy the values from. Not modified.
    */
   void set(T other);

   /**
    * Invalidate this object by setting its values to {@link Double#NaN}.
    */
   void setToNaN();

   /**
    * Reset this object values.
    */
   void setToZero();
}
