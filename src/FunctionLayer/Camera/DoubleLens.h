#pragma once

#include "Perspective.h"

struct Lens
{
    double aperture_radius;
    double focus_distance;
};
// store all the information about a light refracting and reflecting at a plane.
// based on a camera world.
class Intersection_plane
{
public:
    Intersection_plane() {}
    Intersection_plane(const Vec3d incident_dir, const Vec3d N, Point3d intersection, double factor)
    {
        this->incident_dir = incident_dir;
        this->N = N;
        this->intersection_point = intersection;
        this->factor = factor;
        this->cos_i = dot(-N, incident_dir);
        double sin_i = sqrt(1 - sqr(cos_i));
        double sin_t = factor * sin_i;
        if (sin_t >= 1)
        {
            descend_rate = 0;
            R = 1;
        }
        else
        {
            this->cos_t = sqrt(1 - sqr(sin_t));
            this->refraction();

            // Fresnel'law to calculate the descend rate
            // printf("cos_i:%f,cos_t:%f\n",cos_i,cos_t);
            assert(cos_i >= 0);
            assert(cos_t >= 0);
            double r_s = (factor * cos_i / cos_t - 1) / (factor * cos_i / cos_t + 1);
            double R_s = r_s * r_s;
            double r_t = (1 - factor * cos_t / cos_i) / (1 + factor * cos_t / cos_i);
            double R_t = r_t * r_t;
            this->R = (R_s + R_t) / 2;
            this->descend_rate = 1 - R;
        }
        this->reflection();
        // printf("r_s:%f,r_t:%f,R:%f\n", r_s, r_t, R);
    }
    void refraction()
    {
        refract_dir = factor * incident_dir + (factor * cos_i - cos_t) * N;
    }
    void reflection()
    {
        reflect_dir = normalize(incident_dir - 2 * dot(incident_dir, N) * N);
    }
    Vec3d get_reflect_dir()
    {
        return this->reflect_dir;
    }
    Vec3d get_refract_dir()
    {
        return this->refract_dir;
    }
    Point3d get_intersection_point()
    {
        return this->intersection_point;
    }
    double get_descend_rate()
    {
        return this->descend_rate;
    }

    double get_R()
    {
        return this->R;
    }

private:
    Point3d intersection_point;
    Vec3d incident_dir;
    Vec3d reflect_dir;
    Vec3d refract_dir;
    Vec3d N;
    // incident and reflect
    double cos_i;
    // transmit
    double cos_t;
    // n1 / n2
    double factor;
    // refection rate
    double R;
    // refraction rate
    double descend_rate;
};

class DoubleLensCamera : public PerspectiveCamera
{
public:
    explicit DoubleLensCamera(const Json &json);
    // [[nodiscard]]
    Ray generateRay(const Point2i &filmResolotion, const Point2i &pixelPosition, const CameraSample &sample) const override;
    void rayTraceThroughLens(const Ray &ray, const Point2i &filmResolotion, double &descend_rate, double &R, Point2i &fixelPosition) const override;

private:
    Lens len_0;
    Lens len_1;
    double dist_from_0_to_1;
    double refractive_index_air;
    double refractive_index_len;
};