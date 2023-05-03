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

#include "render_engine.hpp"

#include "image/image.hpp"
#include "ray_tracing/scene/scene_parameter.hpp"
#include "ray_tracing/scene/ray.hpp"
#include "ray_tracing/primitives/intersection_data.hpp"
#include "ray_tracing/scene/anti_aliasing_table.hpp"

#include <cmath>

namespace cpe
{


void render(image& im,scene_parameter const& scene)
{
    // **************************************************************************** //
    //
    // Current Algorithm:
    //
    // For all pixels of the image
    //    Generate ray from the camera toward in the direction of the current pixel
    //    Compute associated color (ray tracing algorithm)
    //    Set the color to the current pixel
    // créer du dv pour ajouter des rayons partant des mêmes pixels déclés de du,dv dans les normales
    //
    //
    // **************************************************************************** //

    camera const& cam = scene.get_camera();

    int const Nx = im.Nx();
    int const Ny = im.Ny();
    int const N_sample = 5;
    anti_aliasing_table aa(N_sample);


    // loop over all the pixels of the image
    for(int kx=0 ; kx<Nx ; ++kx)
    {
        float const u = static_cast<float>(kx)/(Nx-1);
        for(int ky=0 ; ky<Ny ; ++ky)
        {
            float const v = static_cast<float>(ky)/(Ny-1);

            for(int dx=0 ; dx<N_sample ; ++dx)
            {
                for(int dy=0 ; dy<N_sample ; ++dy)
                {

                    float const du = aa.displacement(dx)/(Nx-1); //Nx is the size in pixel in x direction
                    float const dv = aa.displacement(dy)/(Ny-1); //Ny is the size in pixel in y direction

                    float const w = aa.weight(dx,dy);

                    ray const r = ray_generator(cam,u+du,v+dv);
                    color const c = ray_trace(r,scene);
                    im({kx,ky})+=w*c;
                }
            }
            /*
             * float du = u/kx;
            float dv = v/ky;
            // generate ray and compute color
            ray const r = ray_generator(cam,u,v);
            ray const r2 = ray_generator(cam, u+du, v+dv);
            ray const r3 = ray_generator(cam, u-du, v-dv);
            color const c = ray_trace(r,scene);
            color const c2 = ray_trace(r2,scene);
            color const c3 = ray_trace(r3,scene);
            im({kx,ky}) = (c+c2+c3)/3;*/
        }
    }

}


ray ray_generator(camera const& cam,float const u,float const v)
{
    // position of the sample on the screen in 3D
    vec3 const p_screen = screen_position(cam,u,v);

    // vector "camera center" to "screen position"
    vec3 const d = p_screen-cam.center();

    // compute the ray
    ray const r(cam.center(),d);

    return r;
}

color ray_trace(ray const& r,scene_parameter const& scene)
{
    // ********************************************* //
    // ********************************************* //
    //
    // TO DO: Calculer la couleur associee a un rayon dans la scene 3D
    //
    // ********************************************* //
    // ********************************************* //
    //Le code suivant affecte la couleur de base de la premiere intersection trouvee
    //Ce code ne gere pas les reflections.


    intersection_data intersection; //current intersection
    int intersected_primitive = 0;  //current index of intersected primitive
    int const& NombreIntersection = 5;
    int k=0;
    ray r2=ray(r.p0(),r.u());
    color c;

    // creation d'un while nous permettant de verifier
    // la valeur des rayons enoyés et de leur reflexion
    float attenuation = 0.8;
    while (k < NombreIntersection){

        bool const is_intersection = compute_intersection(r2,scene,intersection,intersected_primitive);
        if(is_intersection) //if the ray intersects a primitive
        {
            //return 0.5*color(intersection.normal.x(),intersection.normal.y(), intersection.normal.z())+0.5;
            material mat = scene.get_material(intersected_primitive);

            //if(mat.reflection() < 1)
            c += attenuation* compute_illumination(mat,intersection,scene);
            attenuation *= mat.reflection();
            //return intersection.relative/5;

            // on update la valeur de notre rayon afin de prendre
            // le meme rayon d'entrée

            vec3 direction=r2.u()-2*dot(r2.u(), intersection.normal)*intersection.normal;
            r2=ray(intersection.position, direction);
            r2.offset();
        }
        else
        {
            c += color(0,0,0); //no intersection
            break;
        }
        ++k;
    }

    return c;

}


bool compute_intersection(ray const& r,
                          scene_parameter const& scene,
                          intersection_data& intersection,
                          int& index_intersected_primitive)
{
    // ********************************************* //
    // ********************************************* //
    //
    // TO DO: Calculer l'intersection valide la plus proche
    //        Doit retourner vrai/faux si une intersection est trouvee ou non
    //        Doit mettre a jour intersection et index_intersected_primitive avec l'intersection la plus proche
    //
    // ********************************************* //
    // ********************************************* //

    //Le code suivant renvoie la premiere intersection valide que l'on trouve dans l'ordre de stockage du vecteur
    //Ce code est a modifier afin de trouver la 1ere intersection dans l'espace 3D le long du rayon.

    int const N_primitive = scene.size_primitive();

    bool found_intersection = false;
    int k = 0;

    intersection_data intersection_bis = intersection;

    while(k<N_primitive)
    {
        primitive_basic const & primitive = scene.get_primitive(k);
        bool is_intersection = primitive.intersect(r,intersection_bis);
        if(is_intersection)
        {

            if(intersection.relative>intersection_bis.relative || found_intersection==false)
            {
                found_intersection = true;
                intersection=intersection_bis;
                index_intersected_primitive = k;
            }
        }
        k++;
    }
    return found_intersection;
}


bool is_in_shadow(vec3 const& p,vec3 const& p_light, scene_parameter const& scene)
{
    // ********************************************* //
    //
    // TO DO: Calculer si le point p est dans l'ombre de la lumiere situee en p_light
    //
    // ********************************************* //
    //si depuis p_light jusqu'à p il n'y a pas d'intersection, alors il n'est pas dans l'ombre de p_light
    //transformer p_light en source de lumière
    //3 cas: pas d'intersection false, intersection true
    //point = point d'intersection : décaler le rayon originel et observer ensuite
    vec3 u_light = normalized(p_light-p); //orientation du rayon enter p_light et p
    ray rayon(p,u_light);
    intersection_data intersection;
    int index_intersected_primitive;
    bool inter;
    rayon.offset(); //léger décalage du rayon afin d'éviter le pb de l'énoncé
    inter = compute_intersection(rayon,scene, intersection,index_intersected_primitive); //vérification de l'intersection grâce à la fct codée précédemment
    return inter;


}



color compute_illumination(material const& mat,intersection_data const& intersection,scene_parameter const& scene)
{
    // ********************************************* //
    //
    // TO DO: Mettre en place le calcul d'illumination correct
    //
    //   Pour toutes les lumieres
    //     Si point dans l'ombre
    //       Ajouter a couleur courante une faible part de la couleur du materiau
    //          ex. c += mat.color_object()/10.0f;
    //     Sinon
    //       Calculer illumination au point courant en fonction de la position
    //          de la lumiere et de la camera
    //       Ajouter a couleur courante la contribution suivante:
    //           puissance de la lumiere (L.power)
    //         X couleur de la lumiere (L.c)
    //         X valeur de l'illumination
    //
    // ********************************************* //

    color c;

    vec3 const& p0 = intersection.position;

    //calcul illumination
    //shading_parameter shading(0.2f,0.6f,0.8f,5.0f);
    shading_parameter shading;

    int const N_light = scene.size_light();
    for(int k=0; k<N_light ; ++k)
    {
        light const& L = scene.get_light(k);
        bool const shadow = is_in_shadow(p0,L.p,scene);
        if(shadow)
        {
            c += mat.color_object()/10.0f;
        }
        else
        {
            //color illumination = compute_shading(shading,mat.color_object(),p0,L.p,intersection.normal, scene.get_camera().center());
            c += L.power*L.c*compute_shading(shading,mat.color_object(),p0,L.p,intersection.normal, scene.get_camera().center());
            //c += compute_shading(shading,mat.color_object(),p0,L.p,intersection.normal, scene.get_camera().center());
        }
    }

    return c;

}
}





