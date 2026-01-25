# GlobalState Thread Safety Concerns

## Current Issues (NO SYNCHRONIZATION)

The current implementation has **zero thread safety**. Multiple FreeRTOS tasks reading/writing simultaneously will cause **data races, corruption, and undefined behavior**.

### Critical Race Conditions

1. **Orientation updates:**
   - `imu_task` sets orientation while `comms_task` reads it
   - History vector can be modified while being read

2. **Control outputs:**
   - `magnet_task` reads control while `imu_task` writes
   - No ordering guarantees

3. **Current values:**
   - Multiple tasks may write/read simultaneously
   - Vector reallocation during `push_back()` while reading is catastrophic

4. **History vectors:**
   - Unbounded growth (1000+ entries)
   - Copy-on-every-read is expensive
   - Static return buffer in `getOrientationHistory(int)` causes issues with concurrent calls

## Solutions (Priority Order)

### Level 1: Basic Mutex Protection (ESSENTIAL)

Add a single `FreeRTOS::Mutex` (or `std::mutex` if available) to protect all state:

```cpp
private:
    mutable FreeRTOS_SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
```

Wrap every method:
```cpp
Orientation GlobalState::getOrientation() const {
    xSemaphoreTake(mutex, portMAX_DELAY);
    auto ori = orientation;
    xSemaphoreGive(mutex);
    return ori;
}
```

**Pros:** Simple, eliminates most races
**Cons:** Bottleneck, potential priority inversion, lock contention

### Level 2: Reader-Writer Lock

Use `FreeRTOS::RwLock` or similar (check ESP-IDF availability):

```cpp
xSemaphoreTakeRecursive(readLock, portMAX_DELAY);  // multiple readers
xSemaphoreTake(writeLock, portMAX_DELAY);          // exclusive writer
```

**Pros:** Allows concurrent reads (telemetry/comms can read simultaneously)
**Cons:** More complex, potential deadlock if not careful

### Level 3: Lock-Free Structures

Replace vectors with ring buffers (fixed-size, no allocations):

```cpp
template<typename T, size_t N>
class RingBuffer {
    std::array<T, N> data;
    std::atomic<size_t> writeIdx{0};
    std::atomic<size_t> readIdx{0};
    // ... atomic operations
};
```

**Pros:** No locks needed, minimal overhead, safe from interrupt handlers
**Cons:** More complex, bounded capacity, lost data on overflow

### Level 4: Copy-on-Write (CoW) for Histories

Instead of returning references, use `std::shared_ptr`:

```cpp
std::shared_ptr<const std::vector<Orientation>> getOrientationHistoryCopy() const {
    // Create snapshot under lock, return shared pointer
    // Reader can hold old snapshot while writer creates new one
}
```

**Pros:** Lock-free reads, no stale data
**Cons:** Memory overhead, GC-like behavior (shared_ptr refcounting)

## Immediate Recommendations

### 1. **Fix the static buffer bug**
```cpp
// WRONG - shared across concurrent calls:
const std::vector<Orientation>& GlobalState::getOrientationHistory(int last_n) const {
    static std::vector<Orientation> subset;  // ← NOT THREAD-SAFE!
    subset.clear();
    // ...
}

// BETTER - return by value:
std::vector<Orientation> GlobalState::getOrientationHistorySubset(int last_n) const {
    std::vector<Orientation> subset;
    // ...
    return subset;  // moved (no copy)
}
```

### 2. **Bound history sizes**
```cpp
static constexpr size_t MAX_HISTORY = 500;  // not unlimited

void GlobalState::setOrientation(const Orientation& value) {
    orientation = value;
    orientationHistory.push_back(value);
    if (orientationHistory.size() > MAX_HISTORY) {
        orientationHistory.erase(orientationHistory.begin());
    }
}
```

### 3. **Add priority considerations**
If `imu_task` is high-priority and `comms_task` is low:
- Use priority-aware mutexes (FreeRTOS supports this)
- Avoid long critical sections
- Consider separating mutable/immutable state

### 4. **Interrupt safety**
If any access comes from ISRs:
- Can't use blocking mutexes
- Must use atomic operations or ring buffers
- May need to raise this to `iFromISR()` variants

## Testing Thread Safety

```cpp
// In test/
void test_concurrent_writes() {
    // Launch multiple tasks, each writes 1000 times
    // Verify no corruption or lost updates
}

void test_concurrent_read_write() {
    // Reader task continuously reads
    // Writer task continuously writes
    // Verify no exceptions or crashes
}
```

## ESP32 Specifics

**FreeRTOS Primitives Available:**
- `xSemaphoreCreateMutex()` - binary semaphore for locks
- `xSemaphoreTakeRecursive()` - reentrant locks
- `std::atomic<>` - if C++11 atomics are available
- No built-in RwLock in base FreeRTOS (may be in ESP-IDF extensions)

**ISR Context:**
- If called from ISR, use `FromISR` variants: `xSemaphoreTakeFromISR()`
- Better: avoid blocking calls in ISR entirely

## Summary

| Approach | Effort | Safety | Performance |
|----------|--------|--------|-------------|
| Mutex (Level 1) | Low | Medium | ~80% |
| RwLock (Level 2) | Medium | High | 90% (reads) |
| Lock-Free (Level 3) | High | High | 99% |
| CoW (Level 4) | High | High | 95% |

**Recommendation for MVP:** Start with **Level 1 (Mutex)**, add **bounded histories**, fix the static buffer bug. Plan for **Level 3 (RingBuffer)** for the telemetry hot path.
