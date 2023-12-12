using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace PrincipalCurvatureCore
{
    public class PrincipalCurvatureItem
    {
        public double Distance_Accuracy = 1;

        public List<Point3d> UPts { get; private set; }
        public List<Point3d> VPts { get; private set; }
        public List<double> UValue { get; set; }
        public List<double> VValue { get; set; }
        public List<Point3d> CentralPt { get; set; }
        private PrincipalCurvatureItem()
        {
            UPts = new List<Point3d>();
            VPts = new List<Point3d>();
            CentralPt = new List<Point3d>();
        }
        public PrincipalCurvatureItem(double Accuracy) : this()
        {
            this.Distance_Accuracy = Accuracy;
        }
        public void Reverse(bool _Dir)
        {
            if (_Dir)
                this.UPts.Reverse();
            else
                this.VPts.Reverse();
        }
        /// <summary>
        /// Add a point in to either U or V direction.
        /// </summary>
        /// <param name="_Dir">The direction or U and V</param>
        /// <param name="Pt">The point need to be add into list </param>
        /// <returns> if it's looping return false, else true</returns>
        public bool AddPoint(bool _Dir, Point3d Pt)
        {
            if (_Dir)
            {
                if (UPts.Count == 0)
                {
                    UPts.Add(Pt);
                    return true;
                }
                if (Pt.DistanceTo(UPts.First()) < Distance_Accuracy * 2 && UPts.Count > 3)
                {
                    UPts.Add(Pt);
                    UPts.Add(UPts.First());
                    return false;
                }
                else
                {
                    UPts.Add(Pt);
                    return true;
                }
            }
            else
            {
                if (VPts.Count == 0)
                {
                    VPts.Add(Pt);
                    return true;
                }
                if (Pt.DistanceTo(VPts.First()) < Distance_Accuracy * 2 && VPts.Count > 3)
                {
                    VPts.Add(Pt);
                    VPts.Add(VPts.First());
                    return false;
                }
                else
                {
                    VPts.Add(Pt);
                    return true;
                }
            }
        }
    }
}
