#pragma once

#include <optional>

#include "utils/Json.hpp"

class QuadSurface;

namespace segmentation
{

/**
 * Ensure that a surface has a usable uint16 generation channel and return its
 * authoritative current generation. Legacy surfaces without a channel are
 * seeded from meta.max_gen so the UI stop generation and tracer start
 * generation cannot diverge.
 */
[[nodiscard]] std::optional<int> ensureGenerationChannel(QuadSurface* surface);

/**
 * Apply the GUI-requested generation window after custom parameters have been
 * merged. This deliberately disables rewind overrides so a GUI request can
 * process no more than requestedSteps new frontier generations.
 */
[[nodiscard]] int applyGuiGenerationWindow(
    utils::Json& params,
    int currentGeneration,
    int requestedSteps);

} // namespace segmentation
