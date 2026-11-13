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
  return guard_boundary(config.hand_side, "create hand", [&] {
    // interface_name, hand_side and hand_model are already checked by the HandConfig constructor
    auto core = std::make_shared<HandCore>(config);
    cores_.push_back(core);
    return Hand(core, config.hand_side);
  });
}

void HandManager::destroy(Hand& hand)
{
  guard_boundary(hand.side_, "destroy hand", [&] {
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
  guard_boundary("destroy hands", [&] {
    for (auto& core : cores_) {
      if (core) core->close();
    }
    cores_.clear();
  });
}

}  // namespace aidin_hand2
