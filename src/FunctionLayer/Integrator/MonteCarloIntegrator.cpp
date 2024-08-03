/**
 * @file MonteCarloIntegrator.h
 * @author Zhimin Fan
 * edited by orbitchen at 2022-7-7: apply multithread acceleration and tile generator.
 * @brief Integrators
 * @version 0.1
 * @date 2022-05-06
 *
 * @copyright NJUMeta (c) 2022
 * www.njumeta.com
 *
 */

#include <thread>
#include "MonteCarloIntegrator.h"

MonteCarloIntegrator::MonteCarloIntegrator(
    std::shared_ptr<Camera> _camera,
    std::unique_ptr<Film> _film,
    std::unique_ptr<TileGenerator> _tileGenerator,
    std::shared_ptr<Sampler> _sampler,
    int _spp,
    int _renderThreadNum) : Integrator(_camera, std::move(_film)),
                            tileGenerator(std::move(_tileGenerator)),
                            sampler(_sampler),
                            spp(_spp),
                            renderThreadNum(_renderThreadNum)
{
}

void MonteCarloIntegrator::renderPerThread(std::shared_ptr<Scene> scene)
{
    /**
     * @warning Other part of Integrator uses the original sampler
     *          so I have to fill its vectors here.
     *          In fact every time a fresh sampler is needed we should
     *          use Sampler::clone() to get one.
     */
    static int tileFinished = 0;

    sampler->startPixel({0, 0});
    auto ssampler = sampler->clone(0);
    while (true)
    {
        auto optionalTile = tileGenerator->generateNextTile();
        if (optionalTile == std::nullopt)
            break;
        auto tile = optionalTile.value();

        for (auto it = tile->begin(); it != tile->end(); ++it)
        {
            auto pixelPosition = *it;

            const auto &cam = *this->camera;
            /**
             * @bug Sampler is NOT designed for multi-threads, need copy for each thread.
             *      Sampler::clone() will return a Sampler copy, only with same sampling
             *      strategy, random numbers are not guaranteed to be identical.
             */
            // sampler->startPixel(pixelPosition);
            ssampler->startPixel(pixelPosition);
            for (int i = 0; i < spp; i++)
            {
                Ray ray = cam.generateRay(film->getResolution(), pixelPosition, ssampler->getCameraSample());
                if (!(ray.direction.x == 0 && ray.direction.y == 0 && ray.direction.z == 0))
                {

                    auto L = Li(ray, scene);
                    double descend_rate = 1;
                    double R = 0;
                    Point2i Flare_pixelPosition;

                    cam.rayTraceThroughLens(ray, film->getResolution(), descend_rate, R, Flare_pixelPosition);

                    film->deposit(pixelPosition, L * descend_rate);
                    if (R > 0)
                    {
                        // printf("(%d,%d)/(%d,%d),R:%f\n",Flare_pixelPosition.x,Flare_pixelPosition.y,pixelPosition.x,pixelPosition.y,R);
                        // assert(R <= 1);
                        // assert(Flare_pixelPosition.x / film->getResolution().x < 1);
                        // assert(Flare_pixelPosition.y / film->getResolution().y < 1);
                        film->deposit(Flare_pixelPosition, L * R);
                    }
                    // film->deposit(pixelPosition, L);
                    // printf("start deposit reflected ray\n");
                    /**
                     * @warning spp used in this for loop belongs to Integrator.
                     *          It is irrelevant with spp passed to Sampler.
                     *          And Sampler has no sanity check for subscript
                     *          of sample vector. Error may occur if spp passed to
                     *          Integrator is bigger than which passed to Sampler.
                     */
                }
                ssampler->nextSample();
            }
        }

        //* Finish one tile rendering
        if (++tileFinished % 5)
        {
            printProgress((float)tileFinished / tileGenerator->tileCount);
        }
    }
}

void MonteCarloIntegrator::render(std::shared_ptr<Scene> scene)
{
    std::vector<std::thread> threads;
    for (int i = 0; i < renderThreadNum; i++)
    {
        threads.push_back(std::thread(&MonteCarloIntegrator::renderPerThread, this, scene));
    }

    for (int i = 0; i < renderThreadNum; i++)
    {
        threads[i].join();
    }

    printProgress(1.f);
}

double MonteCarloIntegrator::randFloat()
{
    // Get a random number WITHOUT using MonteCarloIntegrator::sampler
    return rand() * 1.0 / RAND_MAX; // todo: better solution
}
