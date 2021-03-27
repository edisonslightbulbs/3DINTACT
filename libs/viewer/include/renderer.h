#ifndef RENDERER_H
#define RENDERER_H

#include <memory>
#include <mutex>
#include <vector>

namespace renderer {
void render(std::mutex& m, int pointNum,
    const std::shared_ptr<std::vector<float>>& sptr_points);
};

#endif /* RENDERER_H */
