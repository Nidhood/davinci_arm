#pragma once

namespace davinci_arm::models {

enum class RecordingState {
    Idle,
    Recording,
    Stopped,
    Completed
};

} // namespace davinci_arm::models