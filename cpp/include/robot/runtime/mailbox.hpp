#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace robot::runtime {

/**
 * Fixed-capacity single-process mailbox.
 *
 * The final firmware will use a bounded inter-core handoff. This mailbox keeps
 * that contract explicit in code now: finite capacity, predictable overwrite
 * behavior chosen by the caller, and no heap allocation.
 */
template <typename T, std::size_t Capacity>
class Mailbox {
 public:
  static_assert(Capacity > 0, "Mailbox capacity must be greater than zero");

  class SpinGuard {
   public:
    explicit SpinGuard(std::atomic_flag& lock) noexcept : lock_(lock) {
      while (lock_.test_and_set(std::memory_order_acquire)) {
      }
    }

    ~SpinGuard() { lock_.clear(std::memory_order_release); }

   private:
    std::atomic_flag& lock_;
  };

  [[nodiscard]] bool Push(const T& value) noexcept {
    SpinGuard guard(lock_);
    if (size_ >= Capacity) {
      return false;
    }
    storage_[tail_] = value;
    tail_ = (tail_ + 1) % Capacity;
    ++size_;
    return true;
  }

  [[nodiscard]] std::optional<T> Pop() noexcept {
    SpinGuard guard(lock_);
    if (size_ == 0) {
      return std::nullopt;
    }
    T value = storage_[head_];
    head_ = (head_ + 1) % Capacity;
    --size_;
    return value;
  }

  [[nodiscard]] bool ReplaceLatest(const T& value) noexcept {
    SpinGuard guard(lock_);
    if (size_ == 0) {
      storage_[tail_] = value;
      tail_ = (tail_ + 1) % Capacity;
      ++size_;
      return true;
    }
    const std::size_t latest_index = tail_ == 0 ? Capacity - 1 : tail_ - 1;
    storage_[latest_index] = value;
    return true;
  }

  [[nodiscard]] std::size_t size() const noexcept {
    SpinGuard guard(lock_);
    return size_;
  }

  [[nodiscard]] bool empty() const noexcept {
    SpinGuard guard(lock_);
    return size_ == 0;
  }

 private:
  std::array<T, Capacity> storage_{};
  std::size_t head_{0};
  std::size_t tail_{0};
  std::size_t size_{0};
  mutable std::atomic_flag lock_ = ATOMIC_FLAG_INIT;
};

}  // namespace robot::runtime
