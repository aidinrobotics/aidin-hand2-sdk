[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand_manager.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandManager`](#handmanager) | class | Move-only type that creates and owns a hand's resources |

## `HandManager`

The manager never opens a CAN socket itself. Connection and control go through the [`Hand`](hand.md#hand)
that [`create()`](#handmanagercreate) returns. When the manager goes away, or you call [`destroy()`](#handmanagerdestroy), every handle to
that `HandCore` becomes invalid.

| Member | Description |
|---|---|
| [`HandManager()`](#handmanagerhandmanager) | Create an empty manager, or move from another |
| [`~HandManager()`](#handmanagerhandmanager-1) | Stop and close every hand it owns |
| [`operator=`](#handmanageroperator) | Move assignment |
| [`create()`](#handmanagercreate) | Validate a config and create a hand |
| [`destroy()`](#handmanagerdestroy) | Close one hand |
| [`destroy_all()`](#handmanagerdestroy_all) | Close every hand it owns |

### `HandManager::HandManager()`

```cpp
HandManager();
HandManager(HandManager&& other) noexcept;
HandManager(const HandManager&) = delete;
```

The default constructor makes a manager with no hands. The move constructor takes over the hands
`other` held, leaving `other` empty. Copying is disabled.

### `HandManager::~HandManager()`

```cpp
~HandManager();
```

Stops and closes every hand it owns, then releases the resources. A connected hand goes through
the shutdown sequence.

### `HandManager::operator=`

```cpp
HandManager& operator=(HandManager&& other) noexcept;
HandManager& operator=(const HandManager&) = delete;
```

Closes every hand it holds and takes over `other`'s, leaving `other` empty.

**Returns**        ｜ A reference to itself

### `HandManager::create()`

```cpp
[[nodiscard]] Hand create(const HandConfig& config);
```

Validates the config and creates one hand. The SDK opens no CAN socket at this point; connect
through the returned handle with [`connect()`](hand.md#handconnect).

**Parameters**     ｜ `config` — the settings for the hand to create<br>
**Returns**        ｜ A [`Hand`](hand.md#hand) pointing at the new `HandCore`<br>
**Throws**         ｜ `InvalidArgument` — the config holds an invalid value

### `HandManager::destroy()`

```cpp
void destroy(Hand& hand);
```

Releases the hand that `hand` refers to. If it is connected, the SDK confirms the stop before it
closes. It releases the hand without throwing even when it cannot confirm the stop, and it leaves
a critical log at the moment the confirmation fails.

**Parameters**     ｜ `hand` — a handle to the hand to close<br>
**Postconditions** ｜ Every [`Hand`](hand.md#hand) pointing at the same `HandCore` becomes invalid

### `HandManager::destroy_all()`

```cpp
void destroy_all();
```

Closes every hand it owns and releases the resources.

**Postconditions** ｜ The manager is empty and all existing handles are invalid

---
