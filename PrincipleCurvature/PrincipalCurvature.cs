using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace PrincipalCurvatureCore
{
    public static class PrincipalCurvature
    {
        public static double AngleSpectrum = Math.PI / 60;
        public static double AnglePrecision = Math.PI / 1080;
        private static Vector3d GetDir(BrepFace Srf, (double, double) UVTest, bool Max, out Point3d PointOnSrf)
        {
            var SrfCrv = Srf.CurvatureAt(UVTest.Item1, UVTest.Item2);

            PointOnSrf = SrfCrv.Point;

            if (SrfCrv.Kappa(0) > SrfCrv.Kappa(1))
            {
                if (Max)
                    return SrfCrv.Direction(0);
                else
                    return SrfCrv.Direction(1);
            }
            else
            {
                if (Max)
                    return SrfCrv.Direction(1);
                else
                    return SrfCrv.Direction(0);
            }

        }
        private static Vector3d GetDir(BrepFace Srf, (double, double) UVTest, bool Max, Vector3d CurrentDir, out Point3d PointOnSrf)
        {
            Vector3d Dir = GetDir(Srf, UVTest, Max, out PointOnSrf);
            if (Dir.IsParallelTo(CurrentDir, Math.PI * 0.5) < 0)
                Dir.Reverse();
            return Dir;
        }
        /// <summary>
        /// Get the next and optimal point that most closet to the principal curvature. this program will 
        /// use parallel for looping
        /// </summary>
        /// <param name="Srf"> BrepFace from CurvatureVertices</param>
        /// <param name="UVtest">ref value. Input is the old UV location, and output is new UV location</param>
        /// <param name="Max">true is U, else V</param>
        /// <param name="CurDir">ref value. 
        /// Current direction from CurvatureVertices in while loop, return final and the 
        /// most-principal-curvature-closed vector.</param>
        /// <param name="PointOnSrf">return the next and optimal point on the surface</param>
        /// <param name="Distance_Accuracy">The distance accuracy from input setting</param>
        /// <returns>if final point is in the srf region return true, else false</returns>
        private static bool ParallelAdjustPoint
            (BrepFace Srf, 
            ref (double, double) UVtest, 
            bool Max, 
            ref Vector3d CurDir, 
            out Point3d PointOnSrf, 
            double Distance_Accuracy)
        {
            #region OPTIMALPOINT
            //Get the current point location from provide UV value and the current point vector from U or V
            Vector3d NextDir = GetDir(Srf, UVtest, Max, CurDir, out Point3d CurPt);
            //Get the perpendicular direction from the uv
            Vector3d NextPerDir = GetDir(Srf, UVtest, !Max, out _);
            //using the NextDir and NextPerDir to retrieve the normal direction
            Vector3d Normal = Vector3d.CrossProduct(NextDir, NextPerDir);
            Vector3d AdjustVector;


            //To Gain the new position, point, and uv value which is much closer than Principal curvature
            //by comparing the current "Normal" with the "TestNormal". If the value is much closer to zero,
            //the TestPtOnSrf and PtUV are selected.
            ConcurrentBag<Point3d> ValuePts = new ConcurrentBag<Point3d>();
            ConcurrentBag<double> ValueNor = new ConcurrentBag<double>();
            ConcurrentBag<(double, double)> ValueUV = new ConcurrentBag<(double, double)>();
            ConcurrentBag<Vector3d> ValueDirs = new ConcurrentBag<Vector3d>();

            List<double> AngleRange = new List<double>();
            for (double i = -AngleSpectrum; i <= AngleSpectrum; i += AnglePrecision)
            {
                AngleRange.Add(i);
            }
            var Vec = new Vector3d(CurDir);
            Parallel.For(0, AngleRange.Count, i => 
                {
                    AdjustVector = new Vector3d(NextDir);
                    
                    AdjustVector.Rotate(AngleRange[i], Normal);
                    var DesiredPt = CurPt + AdjustVector * Distance_Accuracy;
                    Point3d TestPtOnSrf;
                    if (PointOnSurfaceEdge(Srf, CurPt, DesiredPt, out TestPtOnSrf, out var PtUV))
                    {
                        var TestNextDir = GetDir(Srf, PtUV, Max, Vec, out _);
                        var TestVerticalDir = GetDir(Srf, PtUV, !Max, out _);
                        var TestNormal = Vector3d.CrossProduct(TestNextDir, TestVerticalDir);
                        var Value = Math.Abs(TestNormal * Normal);

                        ValueNor.Add(Value);
                        ValuePts.Add(TestPtOnSrf);
                        ValueDirs.Add(AdjustVector);
                        ValueUV.Add(PtUV);
                    }
                });

            if (ValueUV.Count == 0)
            {
                var DesiredPt = CurPt + NextDir * Distance_Accuracy;
                return PointOnSurfaceEdge(Srf, CurPt, DesiredPt, out PointOnSrf, out UVtest);
            }
            else
            {
                var ValuePtsArr = ValuePts.ToArray();
                var ValueDirArr = ValueDirs.ToArray();
                var ValueUVArr = ValueUV.ToArray();
                Array.Sort(ValueNor.ToArray(), ValuePtsArr);
                Array.Sort(ValueNor.ToArray(), ValueDirArr);
                Array.Sort(ValueNor.ToArray(), ValueUVArr);
                PointOnSrf = ValuePtsArr[0];
                UVtest = ValueUVArr[0];
                CurDir = ValueDirArr[0];
                return true;
            }
            #endregion
        }
        /// <summary>
        /// Get the next and optimal point that most closet to the principal curvature.
        /// </summary>
        /// <param name="Srf"> BrepFace from CurvatureVertices</param>
        /// <param name="UVtest">ref value. Input is the old UV location, and output is new UV location</param>
        /// <param name="Max">true is U, else V</param>
        /// <param name="CurDir">ref value. 
        /// Current direction from CurvatureVertices in while loop, return final and the 
        /// most-principal-curvature-closed vector.</param>
        /// <param name="PointOnSrf">return the next and optimal point on the surface</param>
        /// <param name="Distance_Accuracy">The distance accuracy from input setting</param>
        /// <returns>if final point is in the srf region return true, else false</returns>
        private static bool AdjustPoint
            (BrepFace Srf,
            ref (double, double) UVtest,
            bool Max,
            ref Vector3d CurDir,
            out Point3d PointOnSrf,
            double Distance_Accuracy)
        {
            #region OPTIMALPOINT
            //Get the current point location from provide UV value and the current point vector from U or V
            Vector3d NextDir = GetDir(Srf, UVtest, Max, CurDir, out Point3d CurPt);
            //Get the perpendicular direction from the uv
            Vector3d NextPerDir = GetDir(Srf, UVtest, !Max, out _);
            //using the NextDir and NextPerDir to retrieve the normal direction
            Vector3d Normal = Vector3d.CrossProduct(NextDir, NextPerDir);
            Vector3d AdjustVector;

            double MinValue = 10;
            Vector3d? FinalDir = null;
            (double, double)? FinalPtUV = null;
            Point3d FinalPt = Point3d.Unset;
            //To Gain the new position, point, and uv value which is much closer than Principal curvature
            //by comparing the current "Normal" with the "TestNormal". If the value is much closer to zero,
            //the TestPtOnSrf and PtUV are selected.
            for (double i = -AngleSpectrum; i <= AngleSpectrum; i += AnglePrecision)
            {
                AdjustVector = new Vector3d(NextDir);
                AdjustVector.Rotate(i, Normal);
                var DesiredPt = CurPt + AdjustVector * Distance_Accuracy;
                Point3d TestPtOnSrf;
                ///Test if the point is inside the surface
                if (PointOnSurfaceEdge(Srf, CurPt, DesiredPt, out TestPtOnSrf, out var PtUV))
                {
                    var TestNextDir = GetDir(Srf, PtUV, Max, CurDir, out _);
                    var TestVerticalDir = GetDir(Srf, PtUV, !Max, out _);
                    var TestNormal = Vector3d.CrossProduct(TestNextDir, TestVerticalDir);
                    var Value = Math.Abs(TestNormal * Normal);
                    ///Getting the value is the closed to 0 or the minimal one;
                    if (MinValue > Value)
                    {
                        MinValue = Value;
                        FinalPtUV = PtUV;
                        FinalDir = AdjustVector;
                        FinalPt = TestPtOnSrf;
                    }
                }
            }
            //All test points are out of the surface return the closed point on the srf and uv.
            if (MinValue == 10)
            {
                var DesiredPt = CurPt + NextDir * Distance_Accuracy;
                return PointOnSurfaceEdge(Srf, CurPt, DesiredPt, out PointOnSrf, out UVtest);
            }
            //Finalise the PointOnSrf, and UVTest with optimal selection, FinalPtUV and FinalPt,
            //gained from the for loop
            else
            {
                PointOnSrf = FinalPt;
                UVtest = FinalPtUV.Value;
                CurDir = FinalDir.Value;
                return true;
            }
            #endregion
        }
        /// <summary>
        /// Test if the point is inside or outside the BrepFace region by given current point 
        /// and desired point which may not locate on the surface
        /// </summary>
        /// <param name="Srf">BrepFace from AdjustPoint</param>
        /// <param name="CurPt">Current Point from AdjustPoint</param>
        /// <param name="DesiredPt">Desired Point from AdjustPoint</param>
        /// <param name="NextPtOnSrf">Adjusting the point on the surface</param>
        /// <param name="PtUV">Gain the real UV value from the NextPtOnSrf</param>
        /// <returns>if test point is in the srf region return true, else false</returns>
        private static bool PointOnSurfaceEdge(BrepFace Srf, Point3d CurPt, Point3d DesiredPt, out Point3d NextPtOnSrf, out (double, double) PtUV)
        {
            Srf.ClosestPoint(DesiredPt, out double u, out double v);
            PtUV = (u, v);
            NextPtOnSrf = Srf.PointAt(u, v);
            var Edges = Srf.Brep.Edges;
            var JoinEdge = Curve.JoinCurves(Edges)[0];
            JoinEdge.ClosestPoint(NextPtOnSrf, out double m);
            if (JoinEdge.PointAt(m).DistanceTo(NextPtOnSrf) > 10)
                return true;

            var LNCrv = new LineCurve(CurPt, DesiredPt);
            var TestSrf = Extrusion.CreateExtrusion(LNCrv, Srf.NormalAt(u, v) * 3);
            foreach (var Edge in Edges)
            {
                var BoolTest = Intersection.CurveBrep(Edge, TestSrf.ToBrep(), 0.01, 0.01, out var t);
                if (t.Length > 0)
                {
                    NextPtOnSrf = Edge.PointAt(t[0]);
                    Srf.ClosestPoint(NextPtOnSrf, out u, out v);
                    PtUV = (u, v);
                    return false;
                }
            }
            TestSrf = Extrusion.CreateExtrusion(LNCrv, Srf.NormalAt(u, v) * -3);
            foreach (var Edge in Edges)
            {
                var BoolTest = Intersection.CurveBrep(Edge, TestSrf.ToBrep(), 0.01, 0.01, out var t);
                if (t.Length > 0)
                {
                    NextPtOnSrf = Edge.PointAt(t[0]);
                    Srf.ClosestPoint(NextPtOnSrf, out u, out v);
                    PtUV = (u, v);
                    return false;
                }
            }
            return true;
        }
        public static PrincipalCurvatureItem CurvatureVertices(BrepFace Srf, double[] uv, double DistAccuracy = 1)
        {
            var Item = new PrincipalCurvatureItem(DistAccuracy);
            if (uv.Length != 2) return null;
            Interval Iu = Srf.Domain(0);
            Interval Iv = Srf.Domain(1);

            uv[0] = uv[0] == 1 ? uv[0] - 0.001 : uv[0];
            uv[1] = uv[1] == 1 ? uv[1] - 0.001 : uv[1];

            uv[0] = uv[0] == 0 ? uv[0] + 0.001 : uv[0];
            uv[1] = uv[1] == 0 ? uv[1] + 0.001 : uv[1];


            var Pu = uv[0] <= 1 && uv[0] >= 0 ? uv[0] * Iu.Length : 0.5 * Iu.Length;
            var Pv = uv[1] <= 1 && uv[1] >= 0 ? uv[1] * Iv.Length : 0.5 * Iv.Length;

            ///U first with true
            (double, double) Testuv = (Pu, Pv);
            ComputePts(Srf, Testuv, true, ref Item);
            ComputePts(Srf, Testuv, false, ref Item);
            //    var UKp = GetDir(Srf, (Pu, Pv), true, out Point3d iniPt);
            //    var UPreDir = UKp;
            //    bool OnEdge = true;

            //    Item.AddPoint(true, iniPt);
            //    Item.CentralPt.Add(iniPt);
            //    int Count = 0;
            //    while (OnEdge && Count < 1000)
            //    { 
            //        OnEdge = AdjustPoint(Srf, ref Testuv, true, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(true, PtOnSrf))
            //        {
            //            goto UFINISH;
            //        }
            //        Count++;
            //    }
            //    UPreDir = UKp * -1;
            //    OnEdge = true;
            //    Testuv = (Pu, Pv);

            //    Item.Reverse(true);
            //    Count = 0;
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = AdjustPoint(Srf, ref Testuv, true, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(true, PtOnSrf))
            //        {
            //            //throw new Exception();
            //            break;
            //        }
            //        Count++;
            //    }

            //UFINISH:

            //    Testuv = (Pu, Pv);
            //    Count = 0;
            //    var VKp = GetDir(Srf, (Pu, Pv), false, out _);
            //    Item.AddPoint(false, iniPt);
            //    var VPreDir = VKp;
            //    OnEdge = true;

            //    Item.CentralPt.Add(iniPt);
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = AdjustPoint(Srf, ref Testuv, false, ref VPreDir, out var PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(false, PtOnSrf))
            //        {
            //            goto VFINISH;
            //        }
            //        Count++;
            //    }
            //    Testuv = (Pu, Pv);
            //    VPreDir = VKp * -1;
            //    OnEdge = true;
            //    Count = 0;
            //    Item.Reverse(false);
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = AdjustPoint(Srf, ref Testuv, false, ref VPreDir, out var PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(false, PtOnSrf))
            //        {
            //            //throw new Exception();
            //            break;
            //        }
            //        Count++;
            //    }

            //VFINISH:
            return Item;
        }
        public static void ComputePts(BrepFace Srf, (double, double) Testuv, bool Max, ref PrincipalCurvatureItem Item)
        {
            var Pu = Testuv.Item1;
            var Pv = Testuv.Item2;
            var UKp = GetDir(Srf, (Pu, Pv), Max, out Point3d iniPt);
            var UPreDir = UKp;
            bool OnEdge = true;
            var DistAccuracy = Item.Distance_Accuracy;
            Item.AddPoint(Max, iniPt);
            Item.CentralPt.Add(iniPt);
            int Count = 0;
            while (OnEdge && Count < 1000)
            {
                OnEdge = AdjustPoint(Srf, ref Testuv, Max, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
                if (!Item.AddPoint(Max, PtOnSrf))
                {
                    return;
                }
                Count++;
            }
            UPreDir = UKp * -1;
            OnEdge = true;
            Testuv = (Pu, Pv);

            Item.Reverse(Max);
            Count = 0;
            while (OnEdge && Count < 1000)
            {
                OnEdge = AdjustPoint(Srf, ref Testuv, Max, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
                if (!Item.AddPoint(Max, PtOnSrf))
                {
                    //throw new Exception();
                    break;
                }
                Count++;
            }
        }
        public static void ParallelComputePts(BrepFace Srf, (double, double) Testuv, bool Max, ref PrincipalCurvatureItem Item)
        {
            var Pu = Testuv.Item1;
            var Pv = Testuv.Item2;
            var UKp = GetDir(Srf, (Pu, Pv), Max, out Point3d iniPt);
            var UPreDir = UKp;
            bool OnEdge = true;
            var DistAccuracy = Item.Distance_Accuracy;
            Item.AddPoint(Max, iniPt);
            Item.CentralPt.Add(iniPt);
            int Count = 0;
            while (OnEdge && Count < 1000)
            {
                OnEdge = ParallelAdjustPoint(Srf, ref Testuv, Max, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
                if (!Item.AddPoint(Max, PtOnSrf))
                {
                    return;
                }
                Count++;
            }
            UPreDir = UKp * -1;
            OnEdge = true;
            Testuv = (Pu, Pv);

            Item.Reverse(Max);
            Count = 0;
            while (OnEdge && Count < 1000)
            {
                OnEdge = ParallelAdjustPoint(Srf, ref Testuv, Max, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
                if (!Item.AddPoint(Max, PtOnSrf))
                {
                    //throw new Exception();
                    break;
                }
                Count++;
            }
        }
        public static PrincipalCurvatureItem ParallelCurvatureVertices(BrepFace Srf, double[] uv, double DistAccuracy = 1)
        {
            var Item = new PrincipalCurvatureItem(DistAccuracy);
            if (uv.Length != 2) return null;
            Interval Iu = Srf.Domain(0);
            Interval Iv = Srf.Domain(1);

            uv[0] = uv[0] == 1 ? uv[0] - 0.001 : uv[0];
            uv[1] = uv[1] == 1 ? uv[1] - 0.001 : uv[1];

            uv[0] = uv[0] == 0 ? uv[0] + 0.001 : uv[0];
            uv[1] = uv[1] == 0 ? uv[1] + 0.001 : uv[1];


            var Pu = uv[0] <= 1 && uv[0] >= 0 ? uv[0] * Iu.Length : 0.5 * Iu.Length;
            var Pv = uv[1] <= 1 && uv[1] >= 0 ? uv[1] * Iv.Length : 0.5 * Iv.Length;

            ///U first with true
            (double, double) Testuv = (Pu, Pv);
            ParallelComputePts(Srf, Testuv, true, ref Item);
            ParallelComputePts(Srf, Testuv, false, ref Item);
        //    var UKp = GetDir(Srf, (Pu, Pv), true, out Point3d iniPt);
        //    var UPreDir = UKp;
        //    bool OnEdge = true;

            //    Item.AddPoint(true, iniPt);
            //    Item.CentralPt.Add(iniPt);
            //    int Count = 0;
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = ParallelAdjustPoint(Srf, ref Testuv, true, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(true, PtOnSrf))
            //        {
            //            goto UFINISH;
            //        }
            //        Count++;
            //    }
            //    UPreDir = UKp * -1;
            //    OnEdge = true;
            //    Testuv = (Pu, Pv);

            //    Item.Reverse(true);
            //    Count = 0;
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = ParallelAdjustPoint(Srf, ref Testuv, true, ref UPreDir, out Point3d PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(true, PtOnSrf))
            //        {
            //            //throw new Exception();
            //            break;
            //        }
            //        Count++;
            //    }

            //UFINISH:

            //    Testuv = (Pu, Pv);
            //    Count = 0;
            //    var VKp = GetDir(Srf, (Pu, Pv), false, out _);
            //    Item.AddPoint(false, iniPt);
            //    var VPreDir = VKp;
            //    OnEdge = true;

            //    Item.CentralPt.Add(iniPt);
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = ParallelAdjustPoint(Srf, ref Testuv, false, ref VPreDir, out var PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(false, PtOnSrf))
            //        {
            //            goto VFINISH;
            //        }
            //        Count++;
            //    }
            //    Testuv = (Pu, Pv);
            //    VPreDir = VKp * -1;
            //    OnEdge = true;
            //    Count = 0;
            //    Item.Reverse(false);
            //    while (OnEdge && Count < 1000)
            //    {
            //        OnEdge = ParallelAdjustPoint(Srf, ref Testuv, false, ref VPreDir, out var PtOnSrf, DistAccuracy);
            //        if (!Item.AddPoint(false, PtOnSrf))
            //        {
            //            //throw new Exception();
            //            break;
            //        }
            //        Count++;
            //    }

            //VFINISH:
            return Item;
        }
        public static IEnumerable<PrincipalCurvatureItem> CurvatureVertices(BrepFace Srf, double DistAccuracy = 1, params double[][] uv)
         => CurvatureVertices(Srf, uv, DistAccuracy);
        public static IEnumerable<PrincipalCurvatureItem> CurvatureVertices(BrepFace Srf, IEnumerable<double[]> uv, double DistAccuracy = 1)
        {
            foreach (double[] Subuv in uv)
            {
                if (Subuv.Length > 2) yield return null;
                else
                    yield return CurvatureVertices(Srf, Subuv, DistAccuracy);
            }
        }
        public static IEnumerable<PrincipalCurvatureItem> ParallelCurvatureVertices(BrepFace Srf, IEnumerable<double[]> uv, double DistAccuracy = 1)
        {
            ConcurrentBag<PrincipalCurvatureItem> Bag = new ConcurrentBag<PrincipalCurvatureItem>();
            Parallel.ForEach(uv, Subuv =>
            {
                if (Subuv.Length == 2)
                {
                    Bag.Add(CurvatureVertices(Srf, Subuv, DistAccuracy));
                }
            });
            return Bag;
        }
    }
}
