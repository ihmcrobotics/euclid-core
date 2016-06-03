package us.ihmc.geometry.matrix.interfaces;

public interface Matrix3DReadOnly
{
   public double getElement(int row, int column);
   public double getM00();
   public double getM01();
   public double getM02();
   public double getM10();
   public double getM11();
   public double getM12();
   public double getM20();
   public double getM21();
   public double getM22();
}