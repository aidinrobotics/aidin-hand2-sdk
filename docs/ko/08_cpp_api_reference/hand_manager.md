[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand_manager.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandManager`](#handmanager) | class | hand 자원을 만들고 소유하는 move-only 타입 |

---

## `HandManager`

manager 자신은 CAN에 연결하지 않습니다. 연결과 제어는 [`create()`](#handmanagercreate)가 돌려준 [`Hand`](hand.md#hand)로
합니다. manager가 사라지거나 [`destroy()`](#handmanagerdestroy)를 부르면 파기된 `HandCore`를 가리키던 `Hand`가 전부 무효가
됩니다.

| Member | Description |
|---|---|
| [`HandManager()`](#handmanagerhandmanager) | 빈 manager를 만들거나 다른 manager에서 이동 |
| [`~HandManager()`](#handmanagerhandmanager-1) | 소유한 hand를 모두 actuator quick stop 후 종료 |
| [`operator=`](#handmanageroperator) | 이동 대입 |
| [`create()`](#handmanagercreate) | config를 검증하고 hand를 만듦 |
| [`destroy()`](#handmanagerdestroy) | hand 하나를 종료 |
| [`destroy_all()`](#handmanagerdestroy_all) | 소유한 hand를 전부 종료 |

---

### `HandManager::HandManager()`

```cpp
HandManager();
HandManager(HandManager&& other) noexcept;
HandManager(const HandManager&) = delete;
```

기본 생성자는 hand가 없는 manager를 만듭니다. 이동 생성자는 `other`가 갖고 있던 hand의
소유권을 가져오고, `other`는 빈 manager로 남습니다. 복사는 막혀 있습니다.

---

### `HandManager::~HandManager()`

```cpp
~HandManager();
```

소유한 hand를 모두 actuator quick stop 후 종료하고 자원을 반납합니다. 연결 중인 것이 있으면 종료
절차를 거칩니다.

---

### `HandManager::operator=`

```cpp
HandManager& operator=(HandManager&& other) noexcept;
HandManager& operator=(const HandManager&) = delete;
```

갖고 있던 hand를 모두 종료하고 `other`의 소유권을 받습니다. `other`는 빈 manager로 남습니다.

**Returns**        ｜ 자기 자신에 대한 참조

---

### `HandManager::create()`

```cpp
[[nodiscard]] Hand create(const HandConfig& config);
```

config를 검증하고 hand 하나를 만듭니다. 이 시점에는 CAN에 연결하지 않으며, 연결은
돌려받은 handle의 [`connect()`](hand.md#handconnect)로 합니다.

**Parameters**     ｜ `config` — 만들 hand의 설정<br>
**Returns**        ｜ 새로 만든 `HandCore`를 가리키는 [`Hand`](hand.md#hand)<br>
**Throws**         ｜ `InvalidArgument` — `config` 값이 유효하지 않을 때

---

### `HandManager::destroy()`

```cpp
void destroy(Hand& hand);
```

`hand`가 가리키는 대상을 종료하고 자원을 반납합니다. 연결 중이면 actuator quick stop을 확인한 뒤
연결을 끊습니다. 확인하지 못해도 예외 없이 반납하고, 확인에 실패한 시점에 critical log가 남습니다.

**Parameters**     ｜ `hand` — 종료할 대상의 handle<br>
**Postconditions** ｜ 같은 `HandCore`를 가리키던 [`Hand`](hand.md#hand)가 모두 무효가 됨

---

### `HandManager::destroy_all()`

```cpp
void destroy_all();
```

소유한 hand를 전부 종료하고 자원을 반납합니다.

**Postconditions** ｜ manager가 비고, 기존 handle이 모두 무효가 됨
