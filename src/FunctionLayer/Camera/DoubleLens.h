#pragma once

#include "Perspective.h"
#include "CoreLayer/Adapter/JsonUtil.h"

struct Lens
{
    double aperture_radius;
    double focus_distance;    
};



class DoubleLensCamera : public PerspectiveCamera {
public:
    explicit DoubleLensCamera(const Json &json);
    [[nodiscard]]
    Ray generateRay(const Point2i &filmResolotion, const Point2i &pixelPosition, const CameraSample &sample) const override;
    Ray rayTraceThroughLens(Ray &ray);
private:
    Lens len_0;
    Lens len_1;
    double dist_from_0_to_1;
    double refractive_index_air;
    double refractive_index_len;
};