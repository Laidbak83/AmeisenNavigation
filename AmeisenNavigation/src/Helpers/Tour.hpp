#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <span>
#include <vector>

#include "../Utils/Vector3.hpp"

namespace Tour {
/// A 2-opt pass costs O(n^2). On grid-like input late passes keep finding tiny improvements (passes 3-8 on ~1000
/// waypoints: 11 ms for 1.4% shorter), so the search stops once a pass shortens the tour by less than this.
constexpr float MIN_PASS_GAIN = 0.005f;

/// Length of the open tour start -> points[order[0]] -> ...
inline float Length(const Vector3& start, std::span<const Vector3> points, const std::vector<int>& order)
{
    float length = 0.0f;
    Vector3 current = start;

    for (const int i : order)
    {
        length += current.DistanceTo(points[static_cast<size_t>(i)]);
        current = points[static_cast<size_t>(i)];
    }

    return length;
}

/// Order points into a short open tour beginning at `start`: nearest neighbour construction improved by
/// 2-opt moves (segment reversals) until a pass gains less than MIN_PASS_GAIN or maxPasses is reached.
/// turnPenalty (distance per radian of turn, 0 = off) makes the construction prefer going straight on: on a grid
/// many neighbours are equally close, without it the tour zigzags between rows. Returns point indices.
inline std::vector<int> Order(const Vector3& start, std::span<const Vector3> points, int maxPasses = 8,
                              float turnPenalty = 0.0f)
{
    const int n = static_cast<int>(points.size());
    std::vector<int> order;
    order.reserve(points.size());

    // Nearest neighbour.
    std::vector<bool> used(points.size(), false);
    Vector3 current = start;
    float headingX = 0.0f;
    float headingY = 0.0f;
    bool hasHeading = false;

    for (int step = 0; step < n; ++step)
    {
        int best = -1;
        float bestScore = 0.0f;

        for (int i = 0; i < n; ++i)
        {
            if (used[static_cast<size_t>(i)])
            {
                continue;
            }

            const Vector3& p = points[static_cast<size_t>(i)];
            const float d = current.DistanceTo(p);
            float score = d;

            if (turnPenalty > 0.0f && hasHeading && d > 1e-4f)
            {
                const float dx = p.x - current.x;
                const float dy = p.y - current.y;
                const float horizontal = std::sqrt(dx * dx + dy * dy);

                if (horizontal > 1e-4f)
                {
                    const float cosine = std::clamp((dx * headingX + dy * headingY) / horizontal, -1.0f, 1.0f);
                    score += turnPenalty * std::acos(cosine);
                }
            }

            if (best < 0 || score < bestScore)
            {
                best = i;
                bestScore = score;
            }
        }

        const Vector3& next = points[static_cast<size_t>(best)];
        const float dx = next.x - current.x;
        const float dy = next.y - current.y;
        const float horizontal = std::sqrt(dx * dx + dy * dy);

        if (horizontal > 1e-4f)
        {
            headingX = dx / horizontal;
            headingY = dy / horizontal;
            hasHeading = true;
        }

        used[static_cast<size_t>(best)] = true;
        order.push_back(best);
        current = next;
    }

    // 2-opt on the open path start -> order[0] -> ... -> order[n-1] (the start stays fixed, the end is free).
    const auto at = [&](int position) -> const Vector3& {
        return position < 0 ? start : points[static_cast<size_t>(order[static_cast<size_t>(position)])];
    };

    float length = Length(start, points, order);

    for (int pass = 0; pass < maxPasses; ++pass)
    {
        float gain = 0.0f;

        for (int i = 0; i < n - 1; ++i)
        {
            for (int j = i + 1; j < n; ++j)
            {
                // Reverse order[i..j]: edges (i-1, i) and (j, j+1) become (i-1, j) and (i, j+1).
                float delta = at(i - 1).DistanceTo(at(j)) - at(i - 1).DistanceTo(at(i));

                if (j + 1 < n)
                {
                    delta += at(i).DistanceTo(at(j + 1)) - at(j).DistanceTo(at(j + 1));
                }

                if (delta < -1e-3f)
                {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                    gain -= delta;
                }
            }
        }

        length -= gain;

        if (gain <= MIN_PASS_GAIN * length)
        {
            break;
        }
    }

    return order;
}
} // namespace Tour
