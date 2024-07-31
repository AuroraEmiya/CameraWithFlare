#include "DoubleLens.h"
#include "CoreLayer/Math/Warp.h"

DoubleLensCamera::DoubleLensCamera(const Json &json) : PerspectiveCamera(json)
{
    this->len_0.aperture_radius = getOptional(json, "aperture_radius_0", 0.1);
    this->len_0.focus_distance = getOptional(json, "focus_distance_0", 10);
    this->len_1.aperture_radius = getOptional(json, "aperture_radius_1", 1);
    this->len_1.focus_distance = getOptional(json, "focus_distance_1", 10);
    this->dist_from_0_to_1 = getOptional(json, "distance_between", 20);
    // refer to wikipedia
    // https://en.wikipedia.org/wiki/List_of_refractive_indices
    // //printf("%f,%f,%f,%f\n", this->len_0.aperture_radius, this->len_0.focus_distance, this->len_1.aperture_radius, this->len_1.focus_distance);
    this->refractive_index_air = 1.000273;
    this->refractive_index_len = 1.458;
}

Ray DoubleLensCamera::generateRay(const Point2i &filmResolotion, const Point2i &pixelPosition, const CameraSample &sample) const
{
    // double x = (double)(pixelPosition.x + sample.xy.x) / filmResolotion.x;
    // double y = (double)(pixelPosition.y + sample.xy.y) / filmResolotion.y;

    // Point3d p_on_film = sampleToFilm * Point3d(x, y, 0);
    // Point3d p_on_focus_plane_0 = p_on_film * (this->len_0.focus_distance / p_on_film.z);

    // Point2d p_on_disk_0 = this->len_0.aperture_radius * SquareToUniformDiskConcentric(sample.lens);
    // Point3d p_on_aperture_0 = {p_on_disk_0.x, p_on_disk_0.y, 0};

    // Vec3d dir = normalize(p_on_focus_plane_0 - p_on_aperture_0);

    // Point3d p_on_aperture_1 = p_on_aperture_0 + dir * (this->dist_from_0_to_1 / dir.z);
    // // theta_1 should be the angle of incident ray;
    // // theta_2 should be the angle of output ray;
    // Vec3d norm_line = normalize(Vec3d(0, 0, -dir.z));
    // double n1_divided_by_n2 = this->refractive_index_air / this->refractive_index_len;
    // double cos_theta_1 = dot(-norm_line, dir);
    // double sin_theta_1 = sqrt(1 - sqr(cos_theta_1));
    // double sin_theta_2 = n1_divided_by_n2 * sin_theta_1;
    // double cos_theta_2 = sqrt(1 - sqr(sin_theta_2));
    // Vec3d incident_dir = n1_divided_by_n2 * dir + (n1_divided_by_n2 * cos_theta_1 - cos_theta_2) * norm_line;
    // return {cameraToWorld * p_on_aperture_1, cameraToWorld * incident_dir};

    double x = (double)(pixelPosition.x + sample.xy.x) / filmResolotion.x;
    double y = (double)(pixelPosition.y + sample.xy.y) / filmResolotion.y;

    Point3d p_on_film = sampleToFilm * Point3d(x, y, 0);

    Point3d p_on_focus_plane_0 = p_on_film * (this->len_0.focus_distance / p_on_film.z);
    Point2d p_on_disk_0 = this->len_0.aperture_radius * SquareToUniformDiskConcentric(sample.lens);
    Point3d p_on_aperture_0 = {p_on_disk_0.x, p_on_disk_0.y, 0};
    Vec3d dir = normalize(p_on_focus_plane_0 - p_on_aperture_0);

    Vec3d N = normalize(Vec3d(0, 0, -dir.z));
    Point3d intersection_point = p_on_aperture_0 + dir * (dist_from_0_to_1 / dir.z);
    double factor = refractive_index_air / refractive_index_len;
    Intersection_plane *plane1 = new Intersection_plane(dir, N, intersection_point, factor);
    // //printf("point (%f,%f,%f) dir:(%f,%f,%f)\n", plane1->get_intersection_point()[0],plane1->get_intersection_point()[1],plane1->get_intersection_point()[2],plane1->get_refract_dir()[0],plane1->get_intersection_point()[1],plane1->get_refract_dir()[2]);
    return {cameraToWorld * plane1->get_intersection_point(), cameraToWorld * plane1->get_refract_dir()};
}

void DoubleLensCamera::rayTraceThroughLens(const Ray &ray, const Point2i &filmResolotion, double &descend_rate, double &R, Point2i &fixelPosition) const
{
    // to debug if the intersection is out of the range
    Point3d centroid_0(0, 0, 0);
    Point3d centroid_1(0, 0, dist_from_0_to_1);
    // first plane on aperture 1
    // printf("1st plane\n");
    Matrix4x4 worldToCamera = cameraToWorld.inverse();
    Vec3d incident_dir = worldToCamera * (-ray.direction);
    Point3d intersection_point = worldToCamera * ray.origin;
    assert(intersection_point.z - centroid_1.z < 0.000001);
    Vec3d N = normalize(Vec3d(0, 0, -incident_dir.z));
    double factor = refractive_index_len / refractive_index_air;
    Intersection_plane *plane_0 = new Intersection_plane(incident_dir, N, intersection_point, factor);
    descend_rate = plane_0->get_descend_rate();

    // second plane on aperture 0
    // printf("2nd plane\n");
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

    // third plane on aperture 1
    // printf("3rd plane\n");
    incident_dir = plane_1->get_reflect_dir();
    intersection_point = plane_1->get_intersection_point() + incident_dir * (dist_from_0_to_1 / incident_dir.z);
    factor = refractive_index_air / refractive_index_len;
    N = normalize(Vec3d(0, 0, -incident_dir.z));
    Intersection_plane *plane_2 = new Intersection_plane(incident_dir, N, intersection_point, factor);
    if (R != 0 && (intersection_point - centroid_1).length() <= len_1.aperture_radius)
        R = R * plane_2->get_R();

    // fourth plane on aperture 0
    // printf("4th plane\n");
    incident_dir = plane_2->get_reflect_dir();
    k = -plane_2->get_intersection_point().z / incident_dir.z;
    intersection_point = plane_2->get_intersection_point() + k * incident_dir;
    factor = refractive_index_len / refractive_index_air;
    N = normalize(Vec3d(0, 0, -incident_dir.z));
    Intersection_plane *plane_3 = new Intersection_plane(incident_dir, N, intersection_point, factor);
    if (R != 0 && (intersection_point - centroid_0).length() <= len_0.aperture_radius)
        R = R * plane_3->get_descend_rate();
    // onto the film
    // printf("onto film\n");

    incident_dir = plane_3->get_refract_dir();
    Point3d origin = plane_3->get_intersection_point();
    Eigen::Matrix3d matrix3d = sampleToFilm.get3x3();
    Eigen::Matrix3d inverse = matrix3d.inverse();
    Eigen::Vector3d _origin(origin[0], origin[1], origin[2]);
    Eigen::Vector3d _dir(incident_dir[0], incident_dir[1], incident_dir[2]);
    _origin = inverse * _origin;
    _dir = inverse * _dir;
    k = _origin[2] / _dir[2];
    double x = _origin[0] + k * _dir[0];
    double y = _origin[1] + k * _dir[1];
    if (R != 0 && 0 < x < 1 && 0 < y < 1)
    {
        x *= filmResolotion.x;
        y *= filmResolotion.y;

        fixelPosition.x = int(floor(x));
        fixelPosition.y = int(floor(y));
    }
}