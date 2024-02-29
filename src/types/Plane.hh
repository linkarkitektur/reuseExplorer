#pragma once

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/plane.hh>
#include <typed-geometry/tg-std.hh>


namespace linkml {

    struct Plane: tg::plane3
    {
        tg::pos3 origin = tg::pos3(0,0,0);

        Plane() :origin(){};

        Plane(float A, float B, float C, float D)
        {   
            auto norm = tg::normalize_safe(tg::vec3(A, B, C));

            normal.x = norm.x;
            normal.y = norm.y;
            normal.z = norm.z;

            dis = -D;

            origin = tg::project(tg::pos3(0,0,0),*this);
        }
        Plane(float A, float B, float C, float D, float x, float y, float z)
        {

            auto norm = tg::normalize_safe(tg::vec3(A, B, C));

            normal.x = norm.x;
            normal.y = norm.y;
            normal.z = norm.z;

            dis = -D;
            origin = tg::pos3(x,y,z);
        }

        tg::pos2 to2d(tg::pos3 p){
            auto mat = this->get_matrix_from_plane();
            mat = tg::inverse(mat);
            auto p_ = mat * (p-(normal*dis));
            return tg::pos2(p_.x, p_.y);
        }
        tg::pos3 to3d(tg::pos2 p){
            auto mat = this->get_matrix_from_plane();
            tg::pos3 p3d = tg::pos3(p.x, p.y, 0);
            return mat * p3d;
        }

        std::vector<tg::pos2> to2d(std::vector<tg::pos3> const& pts){

            auto mat = this->get_matrix_from_plane();
            mat = tg::inverse(mat);

            std::vector<tg::pos2> out;
            for (auto& p: pts){
                auto p_ = mat * (p-(normal*dis));
                out.emplace_back(tg::pos2(p_.x, p_.y));
            }

            // // Move all points closer to origin
            // auto center = (tg::vec2)tg::average(out);
            // std::transform(out.begin(), out.end(),out.begin(), [&](tg::pos2 & p){ return p - center;});

            return out;
        }
        std::vector<tg::pos3> to3d(std::vector<tg::pos2> const& pts){

            auto mat = this->get_matrix_from_plane();

            std::vector<tg::pos3> out;
            for (auto& p: pts){
                tg::pos3 p3d = tg::pos3(p.x, p.y, 0);
                auto p_ = mat * p3d;
                out.emplace_back(tg::pos3(p_.x, p_.y, p_.z));
            }

            // // Move all points closer to origin
            // auto center = (tg::vec3)tg::average(out);
            // std::transform(out.begin(), out.end(),out.begin(), [&](tg::pos3 & p){ return p - center;});

            return out;
        }

        tg::dir3 X(){
            auto vec = 1 - tg::abs(tg::dot( tg::vec3(0,0,1), this->normal)) > 0.5 ?  tg::vec3(0,0,1):  tg::vec3(1,0,0);
            return tg::normalize( tg::project(this->origin+vec, *this)- this->origin);
        }
        tg::dir3 Y(){
            return tg::normalize(tg::cross(this->X(), this->normal));
        }

        tg::mat4 get_matrix_from_plane(){
            
            auto mat = tg::mat4::ones;
            mat.set_col(0, (tg::vec4)this->X());
            mat.set_col(1, (tg::vec4)this->Y());
            mat.set_col(2, (tg::vec4)this->normal);
            
            return mat;

        }
    
    };
}
