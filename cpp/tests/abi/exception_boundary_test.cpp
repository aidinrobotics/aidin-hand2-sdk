// An Exception thrown inside libaidin_hand2.so is caught by its type outside the library
#include <cstdio>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

int main()
{
  HandManager manager;
  Hand hand = manager.create(HandConfig{"can0", HandSide::Right, HandModel::Gen2});

  try {
    hand.run();  // never connected
  } catch (const Exception& error) {
    if (error.kind() != ErrorKind::WrongCallOrder) {
      std::printf("FAIL: run() before connect() threw %s, not WrongCallOrder\n", to_string(error.kind()));
      return 1;
    }
    std::printf("exception_boundary: all checks passed\n");
    return 0;
  } catch (...) {
    std::printf("FAIL: the exception did not match aidin_hand2::Exception\n");
    return 1;
  }
  std::printf("FAIL: run() before connect() did not throw\n");
  return 1;
}
