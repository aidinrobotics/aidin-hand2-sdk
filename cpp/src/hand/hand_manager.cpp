#include <aidin_hand2/hand/hand_manager.hpp>

#include <algorithm>
#include <memory>

#include "hand_core/hand_core.hpp"
#include "types/status.hpp"

namespace aidin_hand2
{

// -------------------------------- Lifetime ----------------------------------

HandManager::HandManager() = default;

HandManager::~HandManager()
{
  for (auto& core : cores_) {
    if (core) core->close();
  }
}

HandManager::HandManager(HandManager&& other) noexcept = default;
HandManager& HandManager::operator=(HandManager&& other) noexcept = default;

// ---------------------------------- Hands -----------------------------------

Hand HandManager::create(const HandConfig& config)
{
  return guard_boundary("create()", [&] {
    // interface_name and hand_side are already checked by the HandConfig constructor
    if (config.control_rate <= 0) {
      throw_error(ErrorCode::InvalidArgument, "Cannot create hand: control_rate must be positive");
    }
    auto core = std::make_shared<HandCore>(config);
    cores_.push_back(core);
    return Hand(core);
  });
}

void HandManager::destroy(Hand& hand)
{
  guard_boundary("destroy()", [&] {
    auto core = hand.core_.lock();
    if (!core) {
      return;
    }
    core->close();
    cores_.erase(std::remove(cores_.begin(), cores_.end(), core), cores_.end());
  });
}

void HandManager::destroy_all()
{
  guard_boundary("destroy_all()", [&] {
    for (auto& core : cores_) {
      if (core) core->close();
    }
    cores_.clear();
  });
}

}  // namespace aidin_hand2
