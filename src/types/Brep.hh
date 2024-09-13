#pragma once
#include "types/Surface_Mesh.hh"

#include <typed-geometry/types/objects/aabb.hh>
#include <typed-geometry/types/pos.hh>


namespace linkml
{
    class Brep
    {
    public:
        using Vertex    = tg::pos3;
        using Vertices = std::vector<Vertex>;
        using Curve3D   = std::vector<tg::pos3>;
        using Curve2D   = std::vector<tg::pos2>;
        using Curves3D  = std::vector<Curve3D>;
        using Curves2D  = std::vector<Curve2D>;

        using Interval = std::tuple<float, float>;

        struct Face{
            int SurfaceIndex =-1;
            int OuterLoopIndex = -1;
            bool OrientationReversed = false;
            int LoopIndices = -1;
        };

        struct Edge{
            int Curve3dIndex = -1;
            std::vector<int> TrimIndices = std::vector<int>();
            int StartIndex = -1;
            int EndIndex = -1;
            bool ProxyCurveIsReversed = false;
            Interval Domain = Interval(0, 1);
        };

        struct Surface{
            int degreeU = 0;
            int degreeV = 0;
            bool rational = true;
            float area = 0;
            std::vector<float> pointData = std::vector<float>();
            int countU = 1;
            int countV = 1;
            tg::aabb3 bbox = tg::aabb3();
            bool closedU = false;
            bool closedV = false;
            Interval domainU = Interval(0, 1);
            Interval domainV = Interval(0, 1);
            std::vector<float> knotsU = std::vector<float>();
            std::vector<float> knotsV = std::vector<float>();
        };

        struct BrepTrim{
            enum BrepTrimType{
                Unknown = 0,
                Boundary = 1,
                Mated = 2,
                Seam = 3,
                Singular = 4,
                CurveOnSurface = 5,
                PointOnSurface = 6,
                Slit = 7
            };
            int EdgeIndex = -1;
            int StartIndex = -1;
            int EndIndex =  -1;
            int FaceIndex = -1;
            int LoopIndex = -1;
            int CurveIndex = -1;
            int IsoStatus = -1;
            BrepTrimType TrimType = BrepTrimType::Unknown;
            bool IsReversed = false;
            Interval Domain = Interval(0, 1);
        };
        
        struct BrepLoop{
            enum BrepLoopType{
                Unknown = 0,
                Outer = 1,
                Inner = 2,
                Slit = 3,
                CurveOnSurface = 4,
                PointOnSurface = 5
            };
            int FaceIndex = -1;
            std::vector<int> TrimIndices = std::vector<int>();
            BrepLoopType Type = BrepLoopType::Unknown;
        };

        using Faces = std::vector<Face>;
        using Edges = std::vector<Edge>;
        using Surfaces = std::vector<Surface>;
        using Trims = std::vector<BrepTrim>;
        using Loops = std::vector<BrepLoop>;


    private:
        Surface_mesh mesh;
    
    public:
        Brep(Surface_mesh const& mesh);
        void save(std::string const& filename) const;
        static Brep load(std::string const& filename);

        double volume() const;
        double area() const;
        bool is_closed() const;
        tg::aabb3 get_bbox() const;
        int get_Orientation() const;
        LinkMesh get_Mesh() const;

        Curves2D get_Curves2D() const;
        Curves3D get_Curves3D() const;

        Edges get_Edges() const;
        Faces get_Faces() const;
        Vertices get_Vertices() const;
        Surfaces get_Surfaces() const;

        Loops get_Loops() const;
        Trims get_Trims() const;


        void display(std::string name = "Mesh", bool show = true) const;
        
    };
    
} // namespace linkml
