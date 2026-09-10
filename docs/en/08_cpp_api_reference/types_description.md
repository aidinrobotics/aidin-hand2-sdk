[← C++ API Reference](../08_cpp_api_reference.md)

# `types/description.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [Constants](#constants) | `constexpr` | Actuator, joint, and tactile counts |
| [`HandSide`](#enum-handside) | enum | Left / right |
| [`Finger`](#enum-finger) | enum | Tactile finger index |

## Constants

Use these constants for array sizes instead of literals.

```cpp
inline constexpr std::size_t kActuatorCount          = 16;  // actuator state, command, gain
inline constexpr std::size_t kJointCount             = 21;  // joint state including passive
inline constexpr std::size_t kActiveJointCount       = 16;  // joint command
inline constexpr std::size_t kPassiveJointCount      = 5;   // one passive q4 per finger
inline constexpr std::size_t kTaskCount              = 15;  // fingertip task space (5 x xyz)
inline constexpr std::size_t kOrientedTaskCount      = 18;  // thumb 6-D + long finger xyz
inline constexpr std::size_t kFingerCount            = 5;   // thumb, index, middle, ring, baby
inline constexpr std::size_t kTactileTaxelsPerFinger = 17;
inline constexpr std::size_t kPalmTactileCount       = 58;  // palm1(20+20) + palm2(18)
inline constexpr std::size_t kPalm1UpperCount        = 20;
inline constexpr std::size_t kPalm1LowerCount        = 20;
inline constexpr std::size_t kPalm2Count             = 18;
```

## enum `HandSide`

```cpp
enum class HandSide { Left, Right };
```

## enum `Finger`

```cpp
enum class Finger { Thumb, Index, Middle, Ring, Baby };   // tactile finger index 0..4
```

## Array conventions

Check that the length and the unit match before using a value.

| Data | Length | Unit |
|---|---:|---|
| Actuator position | 16 | encoder count |
| Actuator velocity | 16 | rpm |
| Actuator current | 16 | mA |
| Actuator effort command · limit | 16 | % of rated current (`1000`=100 %) |
| Active joint command | 16 | rad |
| Joint position | 21 | rad |
| Joint velocity | 21 | rad/s |
| Joint effort | 21 | N·m |

There are 3 index spaces.

- **actuator index** — length 16. The position, the velocity and the current of an actuator, the
  effort command and limit, and the impedance gains
- **active joint index** — length 16. Joint commands. It groups per finger the same way the
  actuator index does, but the joint and the actuator at the same index do not mean the same axis
- **joint index** — length 21. The position, the velocity and the effort of
  [`JointState`](types_state.md#jointstate). It includes the passive joints

### actuator index

| Index | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Finger | T | T | T | T | I | I | I | M | M | M | R | R | R | B | B | B |

### active joint index

| Index | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Finger | T | T | T | T | I | I | I | M | M | M | R | R | R | B | B | B |

### joint index

| Index | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Finger | T | T | T | T | T | I | I | I | I | M | M | M | M | R | R | R | R | B | B | B | B |
| Active joint index | 0 | 1 | 2 | 3 | — | 4 | 5 | 6 | — | 7 | 8 | 9 | — | 10 | 11 | 12 | — | 13 | 14 | 15 | — |

T=Thumb · I=Index · M=Middle · R=Ring · B=Baby. The passive joint is the last slot of each finger
group, it is not a command target, and it arrives only as FK output. For example, active index `4`,
the first joint of the index finger, is `5` in `JointState`.

---
