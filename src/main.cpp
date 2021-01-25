#include <future>
#include <gflags/gflags.h>
#include <mutex>

#include "dbscan.h"
#include "io.h"
#include "knn.h"
#include "logger.h"
#include "point.h"

#include "workers.h"

extern const int PASS = 0;
extern const int FAIL = -3;

std::mutex m;

static void computeKnn(const std::vector<Point>& t_points,
    const Workers& t_workers,
    std::shared_ptr<std::vector<std::vector<Point>>> sptr_neighbours)
{
    /** asynchronous task to knnTasks */
    auto knnTask
        = [t_points, sptr_neighbours](std::vector<Point> worker) mutable {
              m.lock();
              Knn knn(t_points, worker, sptr_neighbours);
              m.unlock();
          };

    /** create list of asynchronous tasks */
    std::vector<std::future<void>> knnTasks;

    /** distribute asynchronous work for each thread */
    for (auto& worker : *t_workers.sptr_work) {
        knnTasks.push_back(std::async(std::launch::async, knnTask, worker));
    }

    /** do asynchronous work */
    {
        Timer timer;
        for (auto& task : knnTasks) {
            task.get();
        }
        LOG(INFO) << "knn computed in: " << timer.getDuration();
    }
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    std::vector<Point> points;

    /** resources */
    const std::string INPUT_FILE = pwd() + "/resources/somedata.txt";
    const std::string KNN_OUTPUT = pwd() + "/build/output_data.txt";
    const std::string OUTPUT_FILE = pwd() + "/build/results.csv";

    /** parse data */
    points = IO::read(points, INPUT_FILE.c_str());

    /** distribute points on threads */
    const int T = 4;
    Workers workers(T, points);

    /** create shared neighbour lists */
    std::shared_ptr<std::vector<std::vector<Point>>> sptr_neighbours;
    sptr_neighbours
        = std::make_shared<std::vector<std::vector<Point>>>(points.size());

    /** knn */
    computeKnn(points, workers, sptr_neighbours);

    /** ID of point of interest and its K neighbours */
    const int ID = 0;
    const int K = 1;

    /** for each point(ID) output its K nearest neighbours */
    IO::write(ID, K, *sptr_neighbours, KNN_OUTPUT);

    /** run dbscan */
    dbscan ds(points);

    /** write graph-able output */
    IO::write(ds.m_points, OUTPUT_FILE);

    return PASS;
}
