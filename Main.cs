using System;
using System.Collections.Generic;

public class Point2D
{
    public double X {get;set;}
    public double Y {get;set;}
    public Point2D()
    {
        X=0d;
        Y=0d;
    }
    public Point2D(double xEntry, double yEntry)
    {
        X=xEntry;
        Y=yEntry;
    }

    public Point2D GetMiddle(Point2D pt)
    {
        Point2D middle = new Point2D
        {
            X = (X+pt.X)/2,
            Y = (Y+pt.Y)/2
        };
        return middle;
    }

    public override string ToString()
    {
        return "X : " + X + " Y : " + Y;
    }
}

public class Vector2D
{
    public double X {get;set;}
    public double Y {get;set;}
    public Vector2D() 
    {
        X=0d;
        Y=0d;
    }
    public Vector2D(double xInput, double yInput) 
    {
        X=xInput;
        Y=yInput;
    }
    public Vector2D(Point2D p1, Point2D p2) 
    {
        X=p2.X-p1.X;
        Y=p2.Y-p1.Y;
    }
    public double DotProduct(Vector2D vec) 
    {
        double result = (X * vec.X) + (Y * vec.Y);
        return result;
    }
    public double GetNorm()
    {
        double norm =Math.Sqrt(Math.Pow(X,2)+Math.Pow(Y,2));
        return norm;
    }
    public double GetAngleWith(Vector2D vec)
    {
        double angle=0;
        //Vector2D vector = new Vector2D(x,y);
        angle = Math.Acos(this.DotProduct(vec) / (this.GetNorm() * vec.GetNorm()));
        return angle;
    }
    public Vector2D Rotate(double angle)
    {
        Vector2D rotatedVec = new Vector2D();
        rotatedVec.X = (X * Math.Cos(angle)) - (Y * Math.Sin(angle)); 
        rotatedVec.Y = (X * Math.Sin(angle)) + (Y * Math.Cos(angle));
        return rotatedVec;
    }
    public Vector2D Normalize()
    {
        Vector2D normalizedVec = new Vector2D();
        normalizedVec.X = X / this.GetNorm();
        normalizedVec.Y = Y / this.GetNorm();
        return normalizedVec;
    }
    public override string ToString()
    {
        return "X : " + X + " Y : " + Y;
    }
}

public class Segment2D
{
    public Point2D StartSegment{get;set;}
    public Point2D EndSegment{get;set;}
    public double M {get;set;}
    public double P {get;set;}
    public Vector2D vDirector = new Vector2D();
    public Vector2D vNormal = new Vector2D();
    public double A {get;set;}
    public double B {get;set;}
    public double C {get;set;}

    public Segment2D(Point2D startSegment, Point2D endSegment)
    {
        StartSegment = startSegment;
        EndSegment = endSegment;
        vDirector.X = endSegment.X-startSegment.X;//-b cartersian eq
        vDirector.Y = endSegment.Y-startSegment.Y;//a cartesian eq
        A = vDirector.Y;
        B = -vDirector.X;
        C = -(vDirector.Y * startSegment.X) - (-vDirector.X * startSegment.Y);//c
        M = -(A/B);
        P = -(C/B);
        vNormal.X = A;
        vNormal.Y = B;
    }
    public override string ToString()
    {
        string display = "StartSegement : " + StartSegment.ToString() + "\n" +
                         "EndSegement : "   + EndSegment.ToString()   + "\n" + 
                         "A : " + vDirector.Y + "\n" + 
                         "B : " + (-vDirector.X) + "\n" + 
                         "C : " + C +"\n"+
                         "M : " + M +"\n"+
                         "P : " + P +"\n";

        return display;
    }
}

public class Base
{
    public Point2D StartBase{get;set;}
    public Point2D EndBase{get;set;}
    public Base()
    {
        StartBase = new Point2D();
        EndBase = new Point2D();
    }
    public Base(Point2D pt1, Point2D pt2)
    {
        StartBase = pt1;
        EndBase = pt2;
    }
    public static Base FindLandingBase(List<Point2D> terrainPoints)
    {
        Point2D lastPoint = new Point2D();
        int cpt =0;
        Base landingBase = new Base();

        foreach(Point2D point in terrainPoints)
        {
            if(lastPoint.Y == point.Y)
            {
                landingBase = new Base(lastPoint,point);
            }
            lastPoint = point;
            cpt+=1;
        }
        return landingBase;
    }
}

public static class FlightPlanner
{
    public static int flightStep = 0;
    public static List<Segment2D> segments = new List<Segment2D>();
    public static List<int> consigneV = new List<int>();
    public static List<int> consigneH = new List<int>();

    public static void SelectConsigneVSign(Segment2D seg , int consV)
    {
        if(seg.StartSegment.Y > seg.EndSegment.Y || seg.StartSegment.X > seg.EndSegment.X)
        {
            consV = -consV;
        }
        if(seg.M > 0)
        {
            consigneV.Add(-consV);//cons for Grottre mauvais coté
        }
        else if(seg.M < 0)
        {
            consigneV.Add(consV);//-cons
        }
    }
    public static void SelectConsigneHSign(Segment2D seg , int consH)
    {
        if(seg.StartSegment.Y > seg.EndSegment.Y)
        {
            consH = -consH;
        }
        if(seg.M > 0)
        {
            consigneH.Add(-consH);//cons
        }
        else if(seg.M < 0)
        {
            consigneH.Add(consH);//-cons
        }
    }
    public static void Display()
    {
        Console.Error.WriteLine("|Flight Planner|");
        Console.Error.WriteLine("Flight Step : " + flightStep);
        Console.Error.WriteLine("Consigne V : " + consigneV[flightStep]);
        Console.Error.WriteLine("Consigne H : " + consigneH[flightStep]);
        Console.Error.WriteLine("");
    }
}

public static class Utils
{
    public static Point2D LineIntersection(Point2D pt1, Point2D pt2, Point2D pt3, Point2D pt4)
    {
        Point2D result = new Point2D();
        double deltaY = (pt1.Y - pt2.Y);
        double deltaX = (pt1.X - pt2.X);
        double a1 =  deltaY / deltaX;
        double b1 = pt1.Y - (a1 * pt1.X);
        double a2 = (pt3.Y - pt4.Y) / (pt3.X - pt4.X);
        double b2 = pt3.Y - (a2 * pt3.X);
        result.X = (b2-b1)/(a1 - a2);
        result.Y = a1 * result.X + b1;
        return result;
    }
    public static bool IsAnIntersection(Point2D intersect)
    {
        bool result = true;
        if(Double.IsInfinity(intersect.X) || Double.IsInfinity(intersect.Y))
        {
            result = false;
        }
        return result;
    }
    public static bool IsIntersectionOnTerrainSegement(Point2D intersect, Point2D pt1, Point2D pt2)
    {
        if(intersect.X > pt1.X && intersect.X < pt2.X && intersect.Y > pt1.Y && intersect.Y < pt2.Y)
        {
            return true;
        }
        else
        {
            return false;
        }

    }
    public static double RadToDeg(double rad)
    {
        return ((rad * 180) / Math.PI);
    }
     public static double DegToRad(double deg)
    {
        return ((deg * Math.PI) / 180);
    }
}

public static class Pid
{
    public const int MAX_LANDER_POWER = 4;
    public const int MIN_LANDER_POWER = 0;

    public static int errorPosV=0;
    public static int errorPosH=0;

    public static int errorSpeedV = 0;
    public static int errorSpeedH = 0;

    public static int errorSpeedVSum = 0;
    public static int errorSpeedHSum = 0;

    public static int errorSpeedVDelta = 0;
    public static int errorSpeedHDelta = 0;

    public static int speedVoutput = 0;
    public static int speedHoutput = 0;

    public static int posVoutput = 0;
    public static int posHoutput = 0;
    

    public static int kp = 1;
    public static double ki = 0.01;
    public static double kd = 0.1;

    public static int lastErrorSpeedV=0;


    //Il faut séparer les vitesses d'avance rapides et lentes
    //Il faut déterminer le signe en fonction de la montée et de la descente

    public static void ComputeErrorPos(Segment2D segment)
    {
        //erreur Position Verticale
        errorPosV = Convert.ToInt32((segment.M * Lander.XPos + segment.P) - Lander.YPos);
        //erreur Position Horizontale
        errorPosH = Convert.ToInt32(((Lander.YPos - segment.P)/segment.M)- Lander.XPos);
    }
    public static void ComputeErrorSpeed(int consigneV, int consigneH)
    {
        //erreur Vitesse Verticale
        errorSpeedV = consigneV - Lander.VSpeed;
        //erreur Vitesse Horizontale
        Console.Error.WriteLine(consigneH);
        errorSpeedH = consigneH - Lander.HSpeed;
    }
    internal static void ComputeHorizontalOutput()
    {
        //Calcul du nouvel angle avec correcteur proportionnel
        speedHoutput = -errorSpeedH * 1;
        posHoutput = -errorPosH * 1;
    }
    internal static void ComputeVerticalOutput()
    {
        //Calcul de la nouvelle puissance avec PID complet
        speedVoutput = Convert.ToInt32(2 * errorSpeedV + errorSpeedVSum * ki + errorSpeedVDelta * kd);
        posVoutput= kp * errorPosV;
    }
    public static void ComputeCommand(Point2D midBase)
    {
        ComputeErrorPos(FlightPlanner.segments[FlightPlanner.flightStep]);
        ComputeErrorSpeed(FlightPlanner.consigneV[FlightPlanner.flightStep], FlightPlanner.consigneH[FlightPlanner.flightStep]);
        ComputeHorizontalOutput();
        ComputeVerticalOutput();
        int p = PowerServitudeSelector(speedVoutput, posVoutput);
        Lander.Power = PowerBoundry(p);
        int r = AngularServitudeSelector(speedHoutput,posHoutput);
        const int angleLimit = 15;
        r = AngularBoundry(r, angleLimit);
        Lander.Rotate = RedressLastMeters(Lander.YPos,midBase.Y,r);
        
        errorSpeedVSum += errorSpeedV; 
        errorSpeedVDelta = errorSpeedV - lastErrorSpeedV;
        lastErrorSpeedV = errorSpeedV;
    }

    public static int PowerBoundry(int powerInput)
    {
        int result=powerInput;
        if (powerInput>MAX_LANDER_POWER)
        {
            result=MAX_LANDER_POWER;
        }
        if (powerInput<MIN_LANDER_POWER)
        {
            result=MIN_LANDER_POWER;
        }
        return result;
    }
    public static int AngularBoundry(int angularInput, int angularLimit)
    {
        int result=angularInput;
        if (angularInput>angularLimit)
        {
            result=angularLimit;
        }
        if (angularInput<-angularLimit)            
        {
            result=-angularLimit;
        }
        return result;
    }
    public static int PowerServitudeSelector(int powerInput1, int powerInput2) //Speed or position servitude
    {
        if (powerInput1 > powerInput2)
        {
            return powerInput1;
        }
        else
        {
            return powerInput2;
        }
    }
    public static int AngularServitudeSelector(int angularInput1, int angularInput2)
    {
        if (angularInput1 > angularInput2)
        {
            return angularInput1;
        }
        else
        {
            return angularInput2;
        }
    }
    public static int RedressLastMeters(int Y, double landingBaseY, int angle)
    {
        int threshold = 100;
        if(Y < (landingBaseY + threshold))
        {
            return 0;
        }
        else
        {
            return angle;
        }
    }
    public static void Display()
    {
        Console.Error.WriteLine
        (
            "|PID Values|" +             "\n" +
            "ErrorPosV = " + errorPosV + "\n" +
            "ErrorPosH = " + errorPosH + "\n" +
            "ErrorSpeedV = " + errorSpeedV + "\n" + 
            "ErrorSpeedH = " + errorSpeedH + "\n"
        );
    }
}

public static class Lander
{
    public static int XPos {get;set;}
    public static int YPos {get;set;}
    public static int HSpeed {get;set;}
    public static int VSpeed {get;set;}
    public static int Fuel {get;set;}
    public static int Rotate {get;set;}
    public static int Power {get;set;}
}

class Player
{
    public static Point2D DetectIntersectBetweenTerrainAndSeg(List<Point2D> terrainPoints, Segment2D seg)
    {
        Point2D lastPoint = new Point2D();
        Point2D intersect = new Point2D();

        //Console.Error.WriteLine(seg.StartSegment.X);
        
        foreach(Point2D point in terrainPoints)
        {
            intersect = Utils.LineIntersection(seg.StartSegment, seg.EndSegment,lastPoint,point);
            if(Utils.IsIntersectionOnTerrainSegement(intersect, lastPoint, point) && Utils.IsAnIntersection(intersect))
            {
                lastPoint = point;
            }
        }
        return intersect;
    }

    static void Main(string[] args)
    {
        string[] inputs;
        int surfaceN = int.Parse(Console.ReadLine()); // the number of points used to draw the surface of Mars.
        Base landingBase = new Base();
        List<Point2D> terrainPoints = new List<Point2D>();

        for (int i = 0; i < surfaceN; i++)
        {
            inputs = Console.ReadLine().Split(' ');
            int landX = int.Parse(inputs[0]); // X coordinate of a surface point. (0 to 6999)
            int landY = int.Parse(inputs[1]); // Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
            terrainPoints.Add(new Point2D(landX,landY));
            //Console.Error.WriteLine("LandX : " + landX + " LandY : " + landY);      
        }

        landingBase = Base.FindLandingBase(terrainPoints);
     
        int frameNumber=0;

        bool isDoneOnce = false;

        Point2D decalage = new Point2D();
     
        // game loop
        while (true)
        {
            inputs = Console.ReadLine().Split(' ');
            Lander.XPos = int.Parse(inputs[0]);
            Lander.YPos = int.Parse(inputs[1]);
            Lander.HSpeed = int.Parse(inputs[2]); // the horizontal speed (in m/s), can be negative.
            Lander.VSpeed = int.Parse(inputs[3]); // the vertical speed (in m/s), can be negative.
            Lander.Fuel = int.Parse(inputs[4]); // the quantity of remaining fuel in liters.
            Lander.Rotate = int.Parse(inputs[5]); // the rotation angle in degrees (-90 to 90).
            Lander.Power = int.Parse(inputs[6]); // the thrust power (0 to 4).

            Point2D intersect = new Point2D();
            Point2D landerPos = new Point2D();
            Point2D midBase = new Point2D();
            midBase = landingBase.StartBase.GetMiddle(landingBase.EndBase);

            
            
            if(frameNumber==0)
            {
                landerPos = new Point2D(Lander.XPos,Lander.YPos);
                Segment2D initSeg =  new Segment2D(midBase,landerPos);
                Console.Error.WriteLine("mid : " + midBase.X);

                intersect = DetectIntersectBetweenTerrainAndSeg(terrainPoints,initSeg);
                //Console.Error.WriteLine("intersect : " + intersect);
                Segment2D seg = new Segment2D(intersect,landerPos);
             
                Vector2D yAxis = new Vector2D(0,1);

                double teta = yAxis.GetAngleWith(seg.vNormal.Rotate(Math.PI));

                int decValue = 0;
                if (midBase.X==2700)
                {
                    decValue = 300;
                }
                else if (midBase.X==4200)
                {
                    decValue = 2600;
                }

                decalage.X = intersect.X - (Math.Sin(teta) * decValue);
                decalage.Y = intersect.Y + (Math.Cos(teta) * decValue);
            
                Segment2D segDec = new Segment2D(decalage,landerPos);
                FlightPlanner.segments.Add(segDec);
                FlightPlanner.SelectConsigneVSign(segDec,10);
                FlightPlanner.SelectConsigneHSign(segDec,20);
            }

            if (Lander.XPos < decalage.X && !isDoneOnce)
            {
                landerPos = new Point2D(Lander.XPos,Lander.YPos);
                FlightPlanner.flightStep+=1;
                Segment2D seg2 = new Segment2D(midBase,landerPos);
                FlightPlanner.segments.Add(seg2);
                FlightPlanner.SelectConsigneVSign(seg2,15);
                FlightPlanner.SelectConsigneHSign(seg2,10);
                FlightPlanner.consigneH[1]=5;
                isDoneOnce = true;
            }
            FlightPlanner.Display();
            Pid.Display();
            Pid.ComputeCommand(midBase);

            // 2 integers: rotate power. rotate is the desired rotation angle (should be 0 for level 1), power is the desired thrust power (0 to 4).
            Console.WriteLine(Lander.Rotate.ToString()+ " " + Lander.Power.ToString());

            frameNumber+=1;
        }
    }
}
