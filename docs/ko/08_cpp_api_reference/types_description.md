[← C++ API Reference](../08_cpp_api_reference.md)

# `types/description.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [상수](#constants) | `constexpr` | actuator·joint·tactile 개수 |
| [`HandSide`](#enum-handside) | enum | 왼손 / 오른손 |
| [`Finger`](#enum-finger) | enum | tactile finger index |

---

## Constants

배열 크기는 숫자 대신 아래 상수를 쓰십시오.

```cpp
inline constexpr std::size_t kActuatorCount          = 16;  // actuator state·command·gain
inline constexpr std::size_t kJointCount             = 21;  // passive 포함 joint state
inline constexpr std::size_t kActiveJointCount       = 16;  // joint command
inline constexpr std::size_t kPassiveJointCount      = 5;   // finger마다 1개인 passive q4
inline constexpr std::size_t kTaskCount              = 15;  // fingertip task-space (5×xyz)
inline constexpr std::size_t kOrientedTaskCount      = 18;  // thumb 6-D + long finger xyz
inline constexpr std::size_t kFingerCount            = 5;   // thumb·index·middle·ring·baby
inline constexpr std::size_t kTactileTaxelsPerFinger = 17;
inline constexpr std::size_t kPalmTactileCount       = 58;  // palm1(20+20) + palm2(18)
inline constexpr std::size_t kPalm1UpperCount        = 20;
inline constexpr std::size_t kPalm1LowerCount        = 20;
inline constexpr std::size_t kPalm2Count             = 18;
```

---

## enum `HandSide`

```cpp
enum class HandSide { Left, Right };
```

---

## enum `Finger`

```cpp
enum class Finger { Thumb, Index, Middle, Ring, Baby };   // tactile finger index 0..4
```

---

## 배열 규약

길이와 단위가 맞는지 확인하고 쓰십시오.

| Data | Length | Unit |
|---|---:|---|
| Actuator position | 16 | encoder count |
| Actuator velocity | 16 | rpm |
| Actuator current | 16 | mA |
| Actuator effort command · limit | 16 | 정격 전류의 0.1% |
| Active joint command | 16 | rad |
| Joint position | 21 | rad |
| Joint velocity | 21 | rad/s |
| Joint effort | 21 | N·m |

index 공간은 세 가지입니다.

- **actuator index** — 길이 16. actuator의 position·velocity·current, effort command·limit,
  impedance gain
- **active joint index** — 길이 16. joint command. actuator index와 finger별 묶음이 같지만, 같은
  index의 joint와 actuator가 같은 회전축을 뜻하지는 않습니다
- **joint index** — 길이 21. [`JointState`](types_state.md#jointstate)의 position·velocity·effort. passive를 포함합니다

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

T=Thumb · I=Index · M=Middle · R=Ring · B=Baby. passive joint는 각 finger 그룹의 마지막
슬롯이고, command 대상이 아니며 FK 결과로만 전달됩니다. 예를 들어 active index `4`(index finger의
첫 joint)는 `JointState`의 `5`입니다.
