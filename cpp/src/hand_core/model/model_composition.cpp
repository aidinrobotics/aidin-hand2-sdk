#include "hand_core/model/model_composition.hpp"

namespace aidin_hand2
{

// No default, so a HandModel without a composition is a -Wswitch warning here
const ModelComposition& composition_of(HandModel model)
{
  switch (model) {
    case HandModel::Gen2: return kGen2Composition;
    case HandModel::Gen2B: return kGen2BComposition;
  }
  return kGen2Composition;
}

}  // namespace aidin_hand2
