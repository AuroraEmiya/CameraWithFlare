#include "DoubleLens.h"
#include "CoreLayer/Math/Warp.h"
void Vprint(Vec3d vector)
{
    printf("(%f,%f,%f)\n", vector[0], vector[1], vector[2]);
}
void Vprint(Point3d vector)
{
    printf("(%f,%f,%f)\n", vector[0], vector[1], vector[2]);
}
DoubleLensCamera::DoubleLensCamera(const Json &json) : PerspectiveCamera(json)
{
    this->len_0.aperture_radius = getOptional(json, "aperture_radius_0", 0.5);
    this->len_0.focus_distance = getOptional(json, "focus_distance_0", 10);
    this->len_1.aperture_radius = getOptional(json, "aperture_radius_1", 1);
    this->len_1.focus_distance = getOptional(json, "focus_distance_1", 10);
    // printf("%f-----------------------------------\n",getOptional(json, "distance_between", 0.5));
    this->dist_from_0_to_1 = getOptional(json, "distance_between", 0.5);
    // refer to wikipedia
    // https://en.wikipedia.org/wiki/List_of_refractive_indices
    // //printf("%f,%f,%f,%f\n", this->len_0.aperture_radius, this->len_0.focus_distance, this->len_1.aperture_radius, this->len_1.focus_distance);
    this->refractive_index_air = getOptional(json, "refractive_index_air", 1.000273);
    this->refractive_index_len = getOptional(json, "refractive_index_len", 1.5);
    // sampleToFilm.printMatrix();
    // printf("sampleToFilm:\n\t(%f,%f,%f)\n\t(%f,%f,%f)\n\t(%f,%f,%f)\n",sampleToFilm)
}

Ray DoubleLensCamera::generateRay(const Point2i &filmResolotion, const Point2i &pixelPosition, const CameraSample &sample) const
{
    //    aperture_1                aperture_0
    // <--()----dist_from_0_to_1----()-----|(film)
    // <----------------------------O-------------(O.z=0)
    // to debug if the intersection is out of the range
    Point3d centroid_0(0, 0, 0);
    Point3d centroid_1(0, 0, dist_from_0_to_1);
    double x = (double)(pixelPosition.x + sample.xy.x) / filmResolotion.x;
    double y = (double)(pixelPosition.y + sample.xy.y) / filmResolotion.y;

    Point3d p_on_film = sampleToFilm * Point3d(x, y, 0);
 
    // same to the calulation in thinlens
    Point3d p_on_focus_plane_0 = p_on_film * (this->len_0.focus_distance / p_on_film.z);
    Point2d p_on_disk_0 = this->len_0.aperture_radius * SquareToUniformDiskConcentric(sample.lens);
    Point3d p_on_aperture_0 = {p_on_disk_0.x, p_on_disk_0.y, 0};
    Vec3d dir = normalize(p_on_focus_plane_0 - p_on_aperture_0);

    // second refraction that happens in aperture_1
    Vec3d N = normalize(Vec3d(0, 0, -dir.z));
    Point3d intersection_point = p_on_aperture_0 + dir * (dist_from_0_to_1 / dir.z);
    if ((intersection_point - centroid_1).length() <= len_1.aperture_radius)
    {
        double factor = refractive_index_air / refractive_index_len;
        Intersection_plane *plane1 = new Intersection_plane(dir, N, intersection_point, factor);
        Point3d origin = plane1->get_intersection_point();
        Vec3d dir = plane1->get_refract_dir();
        delete plane1;
        return {cameraToWorld * origin, cameraToWorld * dir};
    }
    else
    {
        return {cameraToWorld * Point3d(0, 0, 0), cameraToWorld * Vec3d(0, 0, 0)};
    }
    // //printf("point (%f,%f,%f) dir:(%f,%f,%f)\n", plane1->get_intersection_point()[0],plane1->get_intersection_point()[1],plane1->get_intersection_point()[2],plane1->get_refract_dir()[0],plane1->get_intersection_point()[1],plane1->get_refract_dir()[2]);
}

void DoubleLensCamera::rayTraceThroughLens(const Ray &ray, const Point2i &filmResolotion, double &descend_rate, double &R, Point2i &fixelPosition) const
{
    // printf("check for the path\n");
    // to debug if the intersection is out of the range
    Point3d centroid_0(0, 0, 0);
    Point3d centroid_1(0, 0, dist_from_0_to_1);
    // first plane on aperture 1
    Matrix4x4 worldToCamera = cameraToWorld.inverse();

    Vec3d incident_dir = worldToCamera * (-ray.direction);
    Point3d intersection_point = worldToCamera * ray.origin;
    Vec3d N = normalize(Vec3d(0, 0, -incident_dir.z));
    double factor = refractive_index_len / refractive_index_air;
    Intersection_plane *plane_0 = new Intersection_plane(incident_dir, N, intersection_point, factor);
    // refraction light descends based on Fresnel's law
    descend_rate = plane_0->get_descend_rate();

    // second plane on aperture 0
    incident_dir = plane_0->get_refract_dir();
    double k = -plane_0->get_intersection_point().z / incident_dir.z;
    intersection_point = plane_0->get_intersection_point() + k * incident_dir;
    N = normalize(Vec3d(0, 0, -incident_dir.z));
    factor = refractive_index_air / refractive_index_len;
    Intersection_plane *plane_1 = new Intersection_plane(incident_dir, N, intersection_point, factor);
    if ((intersection_point - centroid_0).length() <= len_0.aperture_radius)
        R = descend_rate * plane_1->get_R();
    else
        R = 0;
    descend_rate = descend_rate * plane_1->get_descend_rate();

    //----------------only trace the reflection ray from apertrue_0---------------------------
    // third plane on aperture 1
    incident_dir = plane_1->get_reflect_dir();
    intersection_point = plane_1->get_intersection_point() + incident_dir * (dist_from_0_to_1 / incident_dir.z);
    factor = refractive_index_air / refractive_index_len;
    N = normalize(Vec3d(0, 0, -incident_dir.z));
    Intersection_plane *plane_2 = new Intersection_plane(incident_dir, N, intersection_point, factor);
    if (R != 0 && (intersection_point - centroid_1).length() <= len_1.aperture_radius)
        R = R * plane_2->get_R();
    else
        R = 0;
    // based on the implement of thinlens I reverse the process of sampling from film to the aperture_0
    if (R != 0)
    {

        incident_dir = plane_2->get_reflect_dir();
        Point3d origin = plane_2->get_intersection_point();
        Point3d temp = sampleToFilm * Point3d(0, 0, 0);
        // find the point on the focus plane that incident_dir hits on
        Point3d p_on_focus_plane = origin + ((len_0.focus_distance - origin.z) / incident_dir.z) * incident_dir;
        Vec3d fraction_dir = Point3d(0, 0, 0) - p_on_focus_plane;
        //
        Matrix4x4 inverse = sampleToFilm.inverse();

        k = (temp.z - p_on_focus_plane.z) / fraction_dir.z;
        Point3d p_on_film = p_on_focus_plane + k * fraction_dir;
        Point3d p = inverse * p_on_film;
        double x = p.x;
        double y = p.y;
        if ((x >= 0 && x < 1) && (y >= 0 && y < 1))
        {
            x *= filmResolotion.x;
            y *= filmResolotion.y;
            fixelPosition.x = int(floor(x));
            fixelPosition.y = int(floor(y));
        }
        else
            R = 0;
    }
    delete plane_0;
    delete plane_1;
    delete plane_2;
}