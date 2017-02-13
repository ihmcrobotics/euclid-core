package us.ihmc.geometry.interfaces;

public interface Clearable
{

   /**
    * Tests if this object contains at least one value equal to {@link Double#NaN}.
    *
    * @return {@code true} if this object contains at least one value equal to {@link Double#NaN},
    *         {@code false} otherwise.
    */
   boolean containsNaN();

   /**
    * Invalidate this object by setting its values to {@link Double#NaN}.
    */
   void setToNaN();

   /**
    * Reset this object values.
    */
   void setToZero();

}