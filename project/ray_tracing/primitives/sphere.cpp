/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "sphere.hpp"

#include "intersection_data.hpp"
#include "../scene/ray.hpp"
#include "lib/common/error_handling.hpp"

#include <cmath>
#include <algorithm>

namespace cpe
{

sphere::sphere(vec3 const& center_param,float radius_param)
    :center_data(center_param),radius_data(radius_param)
{}

vec3 const& sphere::center() const
{
    return center_data;
}
float sphere::radius() const
{
    return radius_data;
}

bool sphere::intersect(ray const& ray_param,intersection_data& intersection) const
{

    vec3 const& u = ray_param.u(); // vecteur unitaire

    // ********************************************************** //
    // ********************************************************** //
    //  TO DO:
    //    Calcul d'intersection entre un rayon et une plan
    //
    // Variales:
    //  - Position initiale du rayon: ray_param.p0()
    //  - Vecteur directeur unitaire du rayon: u
    //  - Position du centre de la sphere: center_data
    //  - Rayon de la sphere: radius_data
    //
    // Aide de syntaxe:
    //  - Norme au carre d'un vecteur v: float norme=v.norm2();
    //             ou: float norme=v.dot(v);
    //
    // ********************************************************** //
    // ********************************************************** //


    vec3 const& p0 = ray_param.p0();
    float a=1.;
    float b=2.*dot(p0-center_data,u);
    float c=dot(p0-center_data,p0-center_data)-radius_data*radius_data;
    float delta = b*b-4.*a*c;

    if (delta<0.)
    {
        return false;
    }
    else
    {
        float const r1=(-b-sqrt(delta))/(2*a);
        float const r2=(-b+sqrt(delta))/(2*a);
        if(r1>0. && r2>0.)
        {
            float t=std::min(r1,r2);
            vec3 n=normalized(center_data-(p0+t*u));
            intersection.set(p0+t*u,-n,t);
            return true;
        }
       else if (r1>0. && r2<0.)
        {
            float t=r1;
            vec3 n=normalized(center_data-(p0+t*u));
            intersection.set(p0+t*u,-n,t);
            return true;

        }
        else if (r1<0. && r2>0.)
        {
            float t=r2;
            vec3 n=normalized(center_data-(p0+t*u));
            intersection.set(p0+t*u,-n,t);
            return true;
        }
        else
        {
            return false;
        }


    }
    return false;


}



}
