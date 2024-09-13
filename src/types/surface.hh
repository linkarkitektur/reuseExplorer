#pragma once

#include "types/PointCloud.hh"
#include "types/Surface_Mesh.hh"

#include <pcl/point_types.h>

#include <embree3/rtcore.h>


using Mesh = linkml::Surface_mesh;
using Plane_3 = linkml::Kernel::Plane_3;
using Point_2 = linkml::Kernel::Point_2;


using Ray = RTCRayHit;
using Rays = std::vector<RTCRayHit>;
using Geometries = std::vector<RTCGeometry>;


namespace linkml
{

    class Surface;

    template <typename SurfacePointer>
    class IndciesIterator
    {
        public:
            IndciesIterator(typename std::unordered_map<unsigned int, Mesh::Face_index>::iterator it, SurfacePointer surface) : it(it), surface(surface) {}
            IndciesIterator& operator++() { ++it; return *this; }
            bool operator!=(const IndciesIterator& other) const { return it != other.it; }
            std::pair<unsigned int, pcl::Indices> operator*() const { return std::pair<unsigned int, pcl::Indices>(it->first, surface->GetPoints(it->second)); }
        private:
            typename std::unordered_map<unsigned int, Mesh::Face_index>::iterator it;
            SurfacePointer surface;
    };

    class Surface  
    {
        public:
            Plane_3 plane;


        private:
            // static constexpr auto coords_name = "v:coords";
            static constexpr auto centeroids_name = "f:centeroids";
            static constexpr auto indecies_name = "f:indecies";
            static constexpr auto num_supporting_points_name = "f:num_supporting_points";

            // Serves as filter for valid faces and enables selection through the Embree ID
            std::unordered_map<unsigned int, Mesh::Face_index> valid;

            Mesh mesh;

    public:
        Surface() = default;
        Surface(PointCloud::Cloud::Ptr cloud, pcl::Indices indices, double tile_x_size = 0.4, double tile_y_size = 0.4);
        
        void Create_Embree_Geometry(RTCDevice & device, RTCScene & scene);

        inline Point_3 GetCentroid(unsigned int id) const { 
            return mesh.property_map<Mesh::Face_index, Point_3>(centeroids_name).value()[valid.at(id)]; }
        inline pcl::Indices GetPoints(unsigned int id) const { 
            return mesh.property_map<Mesh::Face_index, pcl::Indices>(indecies_name).value()[valid.at(id)]; }
        inline pcl::Indices GetPoints(Mesh::Face_index f) const { 
            return mesh.property_map<Mesh::Face_index, pcl::Indices>(indecies_name).value()[f]; }
        inline size_t size() const { return valid.size(); }

        auto begin() { return IndciesIterator(valid.begin(), this); }
        auto end() { return IndciesIterator(valid.end(), this); }

        inline Mesh GetMesh() { return mesh; }
        inline Mesh GetMesh() const { return Mesh(mesh); }


    private:

        std::pair<pcl::Indices,size_t> supporting_points(typename Mesh::Face_index face, const Mesh& mesh, const PointCloud::Cloud::ConstPtr cloud, const pcl::Indices indices);
        Point_3 compute_centroid(typename Mesh::Face_index f, const Mesh& mesh) const;

    };

} // namespace linkml



