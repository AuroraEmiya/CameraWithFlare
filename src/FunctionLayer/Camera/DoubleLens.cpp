#include "DoubleLens.h"
#include "CoreLayer/Math/Warp.h"

DoubleLensCamera::DoubleLensCamera(const Json &json) {
    this->len_0.aperture_radius = getOptional(json, "aperture_radius_0", 0.1);
    this->len_0.focus_distance = getOptional(json, "focus_distance_0", 10);
    this->len_1.aperture_radius = getOptional(json, "aperture_radius_1", 0.1);
    this->len_1.focus_distance = getOptional(json, "focus_distance_1", 10);
    this->dist_from_0_to_1 = getOptional(json, "distance_between", 20);
    // refer to wikipedia
    // https://en.wikipedia.org/wiki/List_of_refractive_indices
    this->refractive_index_air = 1.000273;
    this->refractive_index_len = 1.458;
}

Ray DoubleLensCamera::generateRay(const Point2i &filmResolotion, const Point2i &pixelPosition, const CameraSample &sample) const {
    double x = (double)(pixelPosition.x + sample.xy.x) / filmResolotion.x;
    double y = (double)(pixelPosition.y + sample.xy.y) / filmResolotion.y;

    Point3d p_on_film = sampleToFilm * Point3d(x, y, 0);
    Point3d p_on_focus_plane_0 = p_on_film * (this->len_0.focus_distance / p_on_film.z);

    Point2d p_on_disk_0 = this->len_0.aperture_radius * SquareToUniformDiskConcentric(sample.lens);
    Point3d p_on_aperture_0 = {p_on_disk_0.x, p_on_disk_0.y, 0};

    Vec3d dir = normalize(p_on_focus_plane_0 - p_on_aperture_0);

    Point3d p_on_aperture_1 = (p_on_aperture_0 + dir) * (this->dist_from_0_to_1 / dir.z);
    // theta_1 should be the angle of incident ray;
    // theta_2 should be the angle of output ray;
    Vec3d norm_line = normalize(Vec3d(0, 0, -dir.z));
    double n1_divided_by_n2 = this->refractive_index_air / this->refractive_index_len;
    double cos_theta_1 = dot(-norm_line, dir);
    double sin_theta_1 = sqrt(1 - sqr(cos_theta_1));
    double sin_theta_2 = n1_divided_by_n2 * sin_theta_1;
    double cos_theta_2 = sqrt(1 - sqr(sin_theta_2));
    Vec3d output_dir = n1_divided_by_n2 * dir + (n1_divided_by_n2 * cos_theta_1 - cos_theta_2) * norm_line;
    return {cameraToWorld * p_on_aperture_1, cameraToWorld * output_dir};
}

Ray DoubleLensCamera::rayTraceThroughLens(Ray &ray) {
}